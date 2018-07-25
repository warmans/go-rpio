/*

Package rpio provides GPIO access on the Raspberry PI without any need
for external c libraries (ex: WiringPI or BCM2835).

Supports simple operations such as:
- Pin mode/direction (input/output)
- Pin write (high/low)
- Pin read (high/low)
- Pull up/down/off

Example of use:

	rpio.Open()
	defer rpio.Close()

	pin := rpio.Pin(4)
	pin.Output()

	for {
		pin.Toggle()
		time.Sleep(time.Second)
	}

The library use the raw BCM2835 pinouts, not the ports as they are mapped
on the output pins for the raspberry pi

   Rev 1 Raspberry Pi
+------+------+--------+
| GPIO | Phys | Name   |
+------+------+--------+
|   0  |   3  | SDA    |
|   1  |   5  | SCL    |
|   4  |   7  | GPIO 7 |
|   7  |  26  | CE1    |
|   8  |  24  | CE0    |
|   9  |  21  | MISO   |
|  10  |  19  | MOSI   |
|  11  |  23  | SCLK   |
|  14  |   8  | TxD    |
|  15  |  10  | RxD    |
|  17  |  11  | GPIO 0 |
|  18  |  12  | GPIO 1 |
|  21  |  13  | GPIO 2 |
|  22  |  15  | GPIO 3 |
|  23  |  16  | GPIO 4 |
|  24  |  18  | GPIO 5 |
|  25  |  22  | GPIO 6 |
+------+------+--------+

See the spec for full details of the BCM2835 controller:
http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf

*/

package rpio

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"os"
	"reflect"
	"sync"
	"syscall"
	"time"
	"unsafe"

	"github.com/mgutz/ansi"
	"github.com/olekukonko/tablewriter"
)

type Device interface {
	Pin(num uint8, initialDirection Direction, initialPull Pull) *Pin
	PinMode(pin *Pin, direction Direction)
	WritePin(pin *Pin, state State)
	ReadPin(pin *Pin) State
	TogglePin(pin *Pin)
	PullMode(pin *Pin, pull Pull)
	Open() (err error)
	Close() error
}

type Direction uint8
type State uint8
type Pull uint8

// Memory offsets for gpio, see the spec for more details
const (
	bcm2835Base = 0x20000000
	pi1GPIOBase = bcm2835Base + 0x200000
	memLength   = 4096

	pinMask uint32 = 7 // 0b111 - pinmode is 3 bits
)

// Pin direction, a pin can be set in Input or Output mode
const (
	Input  Direction = iota
	Output
)

// State of pin, High / Low
const (
	Low  State = iota
	High
)

// Pull Up / Down / Off
const (
	PullOff  Pull = iota
	PullDown
	PullUp
)

// Arrays for 8 / 32 bit access to memory and a semaphore for write locking
var (
	memlock sync.Mutex
	mem     []uint32
	mem8    []uint8
)

type Pin struct {
	PinNum uint8
	device Device
}

// Set pin as Input
func (p *Pin) Input() {
	p.device.PinMode(p, Input)
}

// Set pin as Output
func (p *Pin) Output() {
	p.device.PinMode(p, Output)
}

// Set pin High
func (p *Pin) High() {
	p.device.WritePin(p, High)
}

// Set pin Low
func (p *Pin) Low() {
	p.device.WritePin(p, Low)
}

// Toggle pin state
func (p *Pin) Toggle() {
	p.device.TogglePin(p)
}

// Set pin Direction
func (p *Pin) Mode(dir Direction) {
	p.device.PinMode(p, dir)
}

// Set pin state (high/low)
func (p *Pin) Write(state State) {
	p.device.WritePin(p, state)
}

// Read pin state (high/low)
func (p *Pin) Read() State {
	return p.device.ReadPin(p)
}

// Set a given pull up/down mode
func (p *Pin) Pull(pull Pull) {
	p.device.PullMode(p, pull)
}

// Pull up pin
func (p *Pin) PullUp() {
	p.device.PullMode(p, PullUp)
}

// Pull down pin
func (p *Pin) PullDown() {
	p.device.PullMode(p, PullDown)
}

// Disable pullup/down on pin
func (p *Pin) PullOff() {
	p.device.PullMode(p, PullOff)
}

//An instance of a physical device must be a singleton because it always changes the same shared memory
var instance *PhysicalDevice

func NewPhysicalDevice() *PhysicalDevice {
	if instance == nil {
		instance = &PhysicalDevice{}
	}
	return instance
}

type PhysicalDevice struct{}

// Pin refers to the bcm2835 pin
func (d *PhysicalDevice) Pin(num uint8, initialDirection Direction, initialPull Pull) *Pin {
	p := &Pin{PinNum: num, device: d}
	d.PinMode(p, initialDirection)
	d.PullMode(p, initialPull)
	return p
}

// PinMode sets the direction of a given pin (Input or Output)
func (d *PhysicalDevice) PinMode(pin *Pin, direction Direction) {

	// Pin fsel register, 0 or 1 depending on bank
	fsel := uint8(pin.PinNum) / 10
	shift := (uint8(pin.PinNum) % 10) * 3

	memlock.Lock()
	defer memlock.Unlock()

	if direction == Input {
		mem[fsel] = mem[fsel] &^ (pinMask << shift)
	} else {
		mem[fsel] = (mem[fsel] &^ (pinMask << shift)) | (1 << shift)
	}

}

// WritePin sets a given pin High or Low
// by setting the clear or set registers respectively
func (d *PhysicalDevice) WritePin(pin *Pin, state State) {

	p := uint8(pin.PinNum)

	// Clear register, 10 / 11 depending on bank
	// Set register, 7 / 8 depending on bank
	clearReg := p/32 + 10
	setReg := p/32 + 7

	memlock.Lock()
	defer memlock.Unlock()

	if state == Low {
		mem[clearReg] = 1 << (p & 31)
	} else {
		mem[setReg] = 1 << (p & 31)
	}

}

// Read the state of a pin
func (d *PhysicalDevice) ReadPin(pin *Pin) State {
	// Input level register offset (13 / 14 depending on bank)
	levelReg := uint8(pin.PinNum)/32 + 13

	if (mem[levelReg] & (1 << uint8(pin.PinNum))) != 0 {
		return High
	}

	return Low
}

// Toggle a pin state (high -> low -> high)
// TODO: probably possible to do this much faster without read
func (d *PhysicalDevice) TogglePin(pin *Pin) {
	switch d.ReadPin(pin) {
	case Low:
		pin.High()
	case High:
		pin.Low()
	}
}

func (d *PhysicalDevice) PullMode(pin *Pin, pull Pull) {
	// Pull up/down/off register has offset 38 / 39, pull is 37
	pullClkReg := uint8(pin.PinNum)/32 + 38
	pullReg := 37
	shift := uint8(pin.PinNum) % 32

	memlock.Lock()
	defer memlock.Unlock()

	switch pull {
	case PullDown, PullUp:
		mem[pullReg] = mem[pullReg]&^3 | uint32(pull)
	case PullOff:
		mem[pullReg] = mem[pullReg] &^ 3
	}

	// Wait for value to clock in, this is ugly, sorry :(
	time.Sleep(time.Microsecond)

	mem[pullClkReg] = 1 << shift

	// Wait for value to clock in
	time.Sleep(time.Microsecond)

	mem[pullReg] = mem[pullReg] &^ 3
	mem[pullClkReg] = 0

}

// Open and memory map GPIO memory range from /dev/mem .
// Some reflection magic is used to convert it to a unsafe []uint32 pointer
func (d *PhysicalDevice) Open() (err error) {
	var file *os.File
	var base int64

	// Open fd for rw mem access; try gpiomem first
	if file, err = os.OpenFile(
		"/dev/gpiomem",
		os.O_RDWR|os.O_SYNC,
		0); os.IsNotExist(err) {
		file, err = os.OpenFile(
			"/dev/mem",
			os.O_RDWR|os.O_SYNC,
			0)
		base = getGPIOBase()
	}

	if err != nil {
		return
	}

	// FD can be closed after memory mapping
	defer file.Close()

	memlock.Lock()
	defer memlock.Unlock()

	// Memory map GPIO registers to byte array
	mem8, err = syscall.Mmap(
		int(file.Fd()),
		base,
		memLength,
		syscall.PROT_READ|syscall.PROT_WRITE,
		syscall.MAP_SHARED)

	if err != nil {
		return
	}

	// Convert mapped byte memory to unsafe []uint32 pointer, adjust length as needed
	header := *(*reflect.SliceHeader)(unsafe.Pointer(&mem8))
	header.Len /= 32 / 8 // (32 bit = 4 bytes)
	header.Cap /= 32 / 8

	mem = *(*[]uint32)(unsafe.Pointer(&header))

	return nil
}

// Close unmaps GPIO memory
func (d *PhysicalDevice) Close() error {
	memlock.Lock()
	defer memlock.Unlock()
	defer func() {
		instance = nil
	}()
	return syscall.Munmap(mem8)
}

// Read /proc/device-tree/soc/ranges and determine the base address.
// Use the default Raspberry Pi 1 base address if this fails.
func getGPIOBase() (base int64) {
	base = pi1GPIOBase
	ranges, err := os.Open("/proc/device-tree/soc/ranges")
	defer ranges.Close()
	if err != nil {
		return
	}
	b := make([]byte, 4)
	n, err := ranges.ReadAt(b, 4)
	if n != 4 || err != nil {
		return
	}
	buf := bytes.NewReader(b)
	var out uint32
	err = binary.Read(buf, binary.BigEndian, &out)
	if err != nil {
		return
	}
	return int64(out + 0x200000)
}

func NewPi3Simulator(renderPins bool) *Pi3Simulator {
	pi := &Pi3Simulator{close: make(chan bool, 1), renderPins: renderPins}
	pi.pins = []*SimulatedPin{
		{Pin: &Pin{PinNum: 0, device: pi}, name: "3.3V PWR", pinType: "power"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "5V PWR", pinType: "power"},
		{Pin: &Pin{PinNum: 2, device: pi}, name: "GPIO 2", pinType: "io"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "5V PWR", pinType: "power"},
		{Pin: &Pin{PinNum: 3, device: pi}, name: "GPIO 3", pinType: "io"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "GND", pinType: "ground"},
		{Pin: &Pin{PinNum: 4, device: pi}, name: "GPIO 4", pinType: "io"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "UARTO TX", pinType: "-"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "GND", pinType: "ground"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "UARTO RX", pinType: "-"},
		{Pin: &Pin{PinNum: 17, device: pi}, name: "GPIO 17", pinType: "io"},
		{Pin: &Pin{PinNum: 18, device: pi}, name: "GPIO 18", pinType: "io"},
		{Pin: &Pin{PinNum: 27, device: pi}, name: "GPIO 27", pinType: "io"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "GND", pinType: "ground"},
		{Pin: &Pin{PinNum: 22, device: pi}, name: "GPIO 22", pinType: "io"},
		{Pin: &Pin{PinNum: 23, device: pi}, name: "GPIO 23", pinType: "io"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "3.3V PWR", pinType: "power"},
		{Pin: &Pin{PinNum: 24, device: pi}, name: "GPIO 24", pinType: "io"},
		{Pin: &Pin{PinNum: 10, device: pi}, name: "GPIO 10", pinType: "io"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "GND", pinType: "ground"},
		{Pin: &Pin{PinNum: 9, device: pi}, name: "GPIO 9", pinType: "io"},
		{Pin: &Pin{PinNum: 25, device: pi}, name: "GPIO 25", pinType: "io"},
		{Pin: &Pin{PinNum: 11, device: pi}, name: "GPIO 11", pinType: "io"},
		{Pin: &Pin{PinNum: 8, device: pi}, name: "GPIO 8", pinType: "io"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "GND", pinType: "ground"},
		{Pin: &Pin{PinNum: 7, device: pi}, name: "GPIO 7", pinType: "io"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "RESERVED", pinType: "-"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "RESERVED", pinType: "-"},
		{Pin: &Pin{PinNum: 5, device: pi}, name: "GPIO 5", pinType: "io"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "GND", pinType: "ground"},
		{Pin: &Pin{PinNum: 6, device: pi}, name: "GPIO 6", pinType: "io"},
		{Pin: &Pin{PinNum: 12, device: pi}, name: "GPIO 12", pinType: "io"},
		{Pin: &Pin{PinNum: 13, device: pi}, name: "GPIO 13", pinType: "io"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "GND", pinType: "ground"},
		{Pin: &Pin{PinNum: 19, device: pi}, name: "GPIO 19", pinType: "io"},
		{Pin: &Pin{PinNum: 16, device: pi}, name: "GPIO 16", pinType: "io"},
		{Pin: &Pin{PinNum: 26, device: pi}, name: "GPIO 26", pinType: "io"},
		{Pin: &Pin{PinNum: 20, device: pi}, name: "GPIO 20", pinType: "io"},
		{Pin: &Pin{PinNum: 0, device: pi}, name: "GND", pinType: "ground"},
		{Pin: &Pin{PinNum: 21, device: pi}, name: "GPIO 21", pinType: "io"},
	}
	return pi
}

type SimulatedPin struct {
	*Pin
	name      string
	state     State
	pull      Pull
	direction Direction
	pinType   string
}

type Pi3Simulator struct {
	renderPins bool
	pins       []*SimulatedPin
	close      chan bool
}

func (d *Pi3Simulator) Pin(num uint8, initialDirection Direction, initialPull Pull) *Pin {
	p := d.p(num)
	p.direction = initialDirection
	p.pull = initialPull
	return p.Pin
}

func (d *Pi3Simulator) PinMode(pin *Pin, direction Direction) {
	d.p(pin.PinNum).direction = direction
}

func (d *Pi3Simulator) WritePin(pin *Pin, state State) {
	d.p(pin.PinNum).state = state
}

func (d *Pi3Simulator) ReadPin(pin *Pin) State {
	return d.pins[int(pin.PinNum)].state
}

func (d *Pi3Simulator) TogglePin(pin *Pin) {
	if d.p(pin.PinNum).state == High {
		d.p(pin.PinNum).state = Low
		return
	}
	d.p(pin.PinNum).state = High
}

func (d *Pi3Simulator) PullMode(pin *Pin, pull Pull) {
	d.p(pin.PinNum).pull = pull
}

func (d *Pi3Simulator) Open() (err error) {
	if d.renderPins {
		go d.render()
	}
	return
}

func (d *Pi3Simulator) Close() error {
	d.close <- true
	return nil
}

func (d *Pi3Simulator) render() {

	table := tablewriter.NewWriter(os.Stdout)
	table.SetColumnAlignment([]int{
		tablewriter.ALIGN_RIGHT,
		tablewriter.ALIGN_RIGHT,
		tablewriter.ALIGN_RIGHT,
		tablewriter.ALIGN_LEFT,
		tablewriter.ALIGN_LEFT,
		tablewriter.ALIGN_RIGHT,
		tablewriter.ALIGN_LEFT,
		tablewriter.ALIGN_LEFT,
		tablewriter.ALIGN_LEFT,
		tablewriter.ALIGN_LEFT,
	})
	table.SetHeader([]string{"PULL", "DIR", "STATE", "NAME", "PIN", "PIN", "NAME", "STATE", "DIR", "PULL"})

	ticker := time.NewTicker(time.Millisecond * 100)
	for {
		select {
		case <-d.close:
			return
		case <-ticker.C:
			table.ClearRows()
			table.SetColumnAlignment([]int{
				tablewriter.ALIGN_RIGHT,
				tablewriter.ALIGN_RIGHT,
				tablewriter.ALIGN_RIGHT,
				tablewriter.ALIGN_RIGHT,
			})

			var row []string
			for physicalPin, pin := range d.pins {
				if physicalPin%2 == 0 {
					row = append(
						row,
						pullString(pin.pull),
						directionString(pin.direction),
						stateString(pin.state),
						pin.name,
						fmt.Sprintf("%d", physicalPin+1),
					)
				} else {
					row = append(
						row,
						fmt.Sprintf("%d", physicalPin+1),
						pin.name,
						stateString(pin.state),
						directionString(pin.direction),
						pullString(pin.pull),
					)
					//next line
					table.Append(row)
					row = []string{}
				}
			}

			print("\033[H\033[2J")
			table.Render()
		}
	}
}

func (d *Pi3Simulator) p(num uint8) *SimulatedPin {
	for _, p := range d.pins {
		if p.Pin.PinNum == num {
			return p
		}
	}
	return nil
}

func stateString(state State) string {
	if state == Low {
		return ansi.Color("LOW", "green")
	}
	return ansi.Color("HIGH", "red")
}

func directionString(dir Direction) string {
	if dir == Input {
		return "INPUT"
	}
	if dir == Output {
		return "OUTPUT"
	}
	return "UNDEF"
}

func pullString(pull Pull) string {
	if pull == PullUp {
		return ansi.Color("UP", "red")
	}
	if pull == PullDown {
		return ansi.Color("DOWN", "green")
	}
	if pull == PullOff {
		return "OFF"
	}
	return "NA"
}
