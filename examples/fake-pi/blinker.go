/*

A blinker example using go-rpio library.
Requires administrator rights to run

Toggles a LED on physical pin 19 (mcu pin 10)
Connect a LED with resistor from pin 19 to ground.

*/

package main

import (
	"fmt"
	"github.com/warmans/go-rpio"
	"os"
	"time"
)

func main() {

	pi := rpio.NewPi3Simulator()

	// Open and map memory to access gpio, check for errors
	if err := pi.Open(); err != nil {
		fmt.Println(err)
		os.Exit(1)
	}
	defer pi.Close()

	pin := pi.Pin(2)

	// Set pin to output mode
	pin.Output()

	// Toggle pin 20 times
	for x := 0; x < 20; x++ {
		pin.Toggle()
		time.Sleep(time.Second / 5)
	}
}
