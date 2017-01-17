/*
Michael McCrea : University of Washington : DXARTS : Jan 2017
mtm5@uw.edu
*/

Headtracker : Arduino
{
	// TODO: write euler coords to bus
	// var <posBus, <writePos = false, <posPlotter;

	*parserClass { ^HeadtrackerParser }

	send { | aString |
		var str;
		str = aString ++ "\n";
		port.putAll(str);
	}

	free { this.close }

	// TODO
	// setHome { parser.setHome }
	// clearHome { parser.clearHome }
}




HeadtrackerParser : ArduinoParser
{
	var asciiInputLine, <charState, <settingHome = false, homeQ = nil;
	var <>postYPR=false, <>postRaw=false, <>postMsgInterval=false, rcvTime, now;

	init {
		rcvTime = Main.elapsedTime;
	}

	/* called from a Routine (inputThread) in Arduino's init  */
	parse {
		asciiInputLine = List(); // hold each line
		charState = nil;
		// start the loop that reads the SerialPort
		loop { this.parseByte(port.read) };
	}

	parseByte { | byte |
		if (byte === 13) {
			// wait for LF
			charState = 13;
		} {
			if (byte === 10) {
				if (charState === 13) {
					// CR/LF encountered, wrap up this line

					if (asciiInputLine.notEmpty) {
						this.dispatch( asciiInputLine );
					};

					// clear the line stream
					asciiInputLine.clear;
					charState = nil;
				};
			} {
				asciiInputLine.add( byte );
			}
		}
	}

	dispatch { |asciiLine|
		var localCopy, split;

		// must copy line so it isn't lost in the fork below once
		// asciiInputLine.clear in parseByte()
		localCopy = List.copyInstance(asciiLine);

		// split = asciiLine.asAscii.split($,);
		split = asciiLine.collect({|x| {x.asInteger.asAscii}.try ? "" }).join.split($,);

		// for testing: post raw messages by line
		postRaw.if{ split.postln };
		// for testing: see how fast messages are coming in
		// posts interval after each newline, so if there's multiple
		// newlines in a message, it will be misleading
		postMsgInterval.if{
			now = Main.elapsedTime;
			postf("msg interval:\t%\n", (now - rcvTime).reciprocal);
			rcvTime = now;
		}

		// if( split.size >= 5 ){
		// 	block{ |break|
		// 		var hexQuat, q, eulerRad;
		//
		// 		hexQuat = split.copyRange(0, 3);
		//
		// 		q = Quaternion( *hexQuat.collect{ |hex|
		// 			if( hex.size == 8 ) {
		// 				this.strToFloat( hex );
		// 			} { break.("hexQuat is the wrong size".error) }
		// 			}
		// 		);
		//
		// 		if( settingHome ) { homeQ = q.conjugate; settingHome = false; };
		//
		// 		eulerRad =  if( homeQ.notNil, { homeQ * q }, { q } ).asEuler;
		//
		// 		arduino.prDispatchMessage( eulerRad );  // forward to arduino's action
		// 		postYPR.if{ this.prPostYPR( eulerRad ) }; // post values
		// 	}
		// };
	}

	setHome { arduino.send("h") }
	// clearHome { homeQ = nil }

	// prPostYPR { |euler| postf("YAW: %\tPITCH: %\tROLL %\n", *euler.raddeg) }

}



/* -- usage --
SerialPort.devices

h = Headtracker( "/dev/tty.usbmodem1461", 115200)

h.parser.postRaw = true
h.parser.postRaw = false
h.parser.postMsgInterval = true
h.parser.postMsgInterval = false

h.send("h") // set home

h.free

// set the action to be performed with every new
// reading of the sensor. yaw, pitch, and roll are
// passed into your action function
a.action = { |y,p,r| [ y,p,r ].raddeg.postln};

a.setHome
// a.clearHome