(
~now = 0;
~cnt = 0;
~times = List();
d = OSCdef(\htracker, {
	|msg, time, addr, recvPort|
	var fs, ypr;
	fs = (time - ~now).reciprocal;
	~now = time;
	~times.add(time);
	~cnt = ~cnt+1;
	/*
	rotate = yaw
	tilt = pitch
	tumble = roll
	*/
	ypr = msg[1..3];
	ypr.round(0.1).do({|v| (v.asString.padLeft(5," ") ++ " ").post }); "\n".post;
	ypr = msg[1..3].degrad;
	~rot =  ypr[0];
	~tilt = ypr[1];
	~tum =  ypr[2];
	// postf("\t % \t %, fs: %\n", time, ~cnt, fs);
}, '/imu'
);
)
// d.free

// n = NetAddr("localhost", 57120)
// n.sendMsg('/imu', 14)

/*
At+BAUDRATE=115200
AT+GATTADDSERVICE=UUID128=00-11-00-11-44-55-66-77-88-99-AA-BB-CC-DD-EE-FF
AT+GATTADDCHAR=UUID128=00-11-22-33-44-55-66-77-88-99-AA-BB-CC-DD-EE-FF,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=2,DATATYPE=3,VALUE=154

*/