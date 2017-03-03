(
~now = 0;
~cnt = 0;
~times = List();
d = OSCdef(\htracker, {
	|msg, time, addr, recvPort|
	var fs, ypr,uivals;
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
	ypr.round(0.1).do({|v| (v.asString.padLeft(5," ") ++ "\t\t").post }); "\n".post;
	uivals = ypr.copy;
	{
		~dials !? {
			~dials[0].value=uivals[0]; // rotate
			~dials[1].value=uivals[1]; // tilt
			~dials[2].value=uivals[2]; // tumble
		}
	}.defer;

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



(
var setupRot, numRots = 3;
setupRot = {|col, i|
	var r;
	r= RotaryView(bounds: Size(120,120).asRect, spec: [0,45].asSpec, innerRadiusRatio: 0.1);
	r.spec = [0, 360, \lin].asSpec;
	// if (i==0) {
	// 	r.spec = [0, 360, \lin].asSpec;
	// } {
	// 	r.spec = [-180, 180, \lin].asSpec;
	// };
	r.startAngle_(0pi).sweepLength_(2pi);
	r.range.fillColor = Color.new(0.9,0.9,0.9);
	r.range.stroke = false;
	r.level.strokeColor = col;
	r.handle.color = col;
	// r.level.fillColor = col.copy.alpha_(0.2);
	r.level.fill = false;
	r.level.stroke = false;
	r.level.strokeType = \outside;
	// r.valueFontColor = col.val_(0.2);
	r.level.strokeWidth = 3;
	r.text.fontSize = 12;
	r.text.align = \center;
	r.ticks.show = true;
	r.ticks.numTicks_(12, 3, false);
	r.action = {|val| val.postln};
	r.bipolar = false;
	r.wrap=true;
	r;
};

~dials = Array.newClear(3);

w = Window(bounds:Rect(100,100, 140*numRots, 190)).front.layout_(
	HLayout(
		*["Rotate", "Tilt", "Tumble"].collect({
			|name, i|
			VLayout(
				StaticText().string_(name).align_(\center).maxHeight_(50),
				{
					var r = setupRot.(Color.hsv(numRots.reciprocal*i, 1,1,1));
					~dials[i] = r;
					r
				}.value
			)
		})
	)
)
)