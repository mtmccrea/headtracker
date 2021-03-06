/* a rotating 3D arrow */


(
// https://en.wikipedia.org/wiki/3D_projection
var win, rot = 0;

~rot = 0;
~tilt = 0;
~tum = 0;

~ax = 0.75; // x pos in the world
~ay = 0.25; // y pos in the world
~bz = 1.2; // distance to screen
~az = ~bz + 0.2; // distance to point from eye


win = Window().front;
UserView(win, win.view.bounds.size.asRect).drawFunc_({ |v|
	var bnds, halfW, halfH, halfD, sqr;
	var arrw_pnts_top, arrw_pnts_bottom, arrw_pnts, arrw_pnts_xf;
	var arrw_pnts_top_xf, arrw_pnts_bottom_xf;

	bnds = v.bounds;
	#halfW, halfH = [bnds.width.half, bnds.height.half];
	// halfD = halfW; // half depth

	// start with centered arrow
	arrw_pnts_top= [
		// top, x, z
		0,1,
		-0.8, 0.2,
		-0.3, 0.2,
		-0.3, -1,
		0.3, -1,
		0.3, 0.2,
		0.8, 0.2,
	].clump(2).collect{|xz|
		var x,z;
		#x,z = xz;
		Cartesian(x, 0.2, z)
	};

	arrw_pnts_bottom = arrw_pnts_top.collect(_ * Cartesian(1, -1, 1));

	arrw_pnts = arrw_pnts_top ++ arrw_pnts_bottom;

	// rot = rot+(pi/50) % 2pi;

	// rotate square
	arrw_pnts = arrw_pnts.collect{ |pnt|
		// pnt.rotate(~tilt) // NOTE:transform swapped to match visually!
		// pnt.asSpherical.rotateXY(~rot).rotateYZ(~tilt).rotateXZ(~tum).asCartesian
		pnt.asSpherical.rotate(~tum).tilt(~tilt).tumble(~rot).asCartesian
		+ (~ax@~ay.neg) // offset position (in normalized coords)
		* Cartesian(halfW, halfH, 1); // scale to window size
	};

	// xformed points from 3D>2D
	// + cart.z; accounts for depth adjusted by rotation
	// (~az is the depth position of the _center_ of shape's rotation)
	arrw_pnts_xf = arrw_pnts.collect{|cart| cart*(~bz/(~az+cart.z))};

	// discard z for 2D
	arrw_pnts_xf = arrw_pnts_xf.collect(_.asPoint);

	arrw_pnts_top_xf = arrw_pnts_xf[.. (arrw_pnts_xf.size.half-1).asInteger];
	arrw_pnts_bottom_xf = arrw_pnts_xf[arrw_pnts_xf.size.half.asInteger..];

	Pen.translate(halfW, halfH); // move to center
	// Pen.moveTo(arrw_pnts_xf[0]);

	// draw top arrow
	Pen.moveTo(arrw_pnts_top_xf[0]);
	arrw_pnts_top_xf.rotate(-1).do{|pnt|
		Pen.lineTo(pnt)
	};
	Pen.fillColor_(Color.gray.alpha_(0.5));
	Pen.fill;

	// draw bottom arrow
	Pen.moveTo(arrw_pnts_bottom_xf[0]);
	arrw_pnts_bottom_xf.rotate(-1).do{|pnt|
		Pen.lineTo(pnt)
	};
	Pen.fillColor_(Color.blue.alpha_(0.5));
	Pen.fill;

	// draw interconnect lines between arrow vertices
	arrw_pnts_top_xf.do{|pnt, i|
		Pen.moveTo(pnt);
		Pen.lineTo(arrw_pnts_bottom_xf[i])
	};

	Pen.stroke;

}).resize_(5).animate_(true).frameRate_(40);
)

(
Window().front.view.layout_(
	VLayout(
		// x position l>r -1>1
		StaticText().string_("x position l>r -1>1"),
		Slider().orientation_(\horizontal)
		.action_({|sl|
			~ax = sl.value.linlin(0,1, -1,1)
		}),
		// y position l>r -1>1
		StaticText().string_("y position l>r -1>1"),
		Slider().orientation_(\vertical).maxWidth_(25)
		.action_({|sl|
			~ay = sl.value.linlin(0,1, -1,1)
		}),

		StaticText().string_("dist to of obj from screen 0>2:"),
		// dist to of obj from screen 0>2
		Slider().orientation_(\horizontal)
		.action_({|sl|
			~az = ~bz + sl.value.linlin(0,1, 0,2).postln;
		}),

		StaticText().string_("dist of eyeball to screen 0.2>3:"),
		// dist of eyeball to screen 0.2>3
		Slider().orientation_(\vertical).maxWidth_(25)
		.action_({|sl|
			~bz = sl.value.linlin(0,1, 0.2,3).postln;
		}),

	)
)
)


~rot = 90.degrad;
~tilt = 45.degrad;
~tum = -45.degrad;
