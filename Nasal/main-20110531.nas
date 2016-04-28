var dialog = gui.Dialog.new("/sim/gui/dialogs/parachutist/config/dialog",
                            "Aircraft/Parachutist/Dialogs/config.xml");

var move_sec = 1.0;		# time to complete all joint motions
var end_simulation_agl = 100; # end run below the height (ft)

var last_time = 0.0;
var poseNode = props.globals.getNode("/fdm/jsbsim/Creare/pose-name", 1);
poseNode.setValue("Box");
var last_pose = "nothing";

var end_of_simulation = 0;


# transform euler angles from a fixed syste to a rotating system
# through an intermediate quaternion step.
var rot_trans = func( fixed ) {
    var rot = {};

    var Rx = fixed.Tx * D2R * 0.5;
    var Ry = fixed.Ty * D2R * 0.5;
    var Rz = fixed.Tz * D2R * 0.5;

    var At = math.cos(Rx) * math.cos(Ry) * math.cos(Rz) - math.sin(Rx) * math.sin(Ry) * math.sin(Rz);
    var Ax = math.cos(Rx) * math.sin(Ry) * math.sin(Rz) + math.sin(Rx) * math.cos(Ry) * math.cos(Rz);
    var Ay = math.cos(Rx) * math.sin(Ry) * math.cos(Rz) - math.sin(Rx) * math.cos(Ry) * math.sin(Rz);
    var Az = math.cos(Rx) * math.cos(Ry) * math.sin(Rz) + math.sin(Rx) * math.sin(Ry) * math.cos(Rz);

    var Px = math.atan2( 2*(At*Ax + Ay*Az), 1 - 2*(Ax*Ax + Ay*Ay) );
    var Py = math.asin( 2*(At*Ay - Az*Ax) );
    var Pz = math.atan2( 2*(At*Az + Ax*Ay), 1 - 2*(Ay*Ay + Az*Az) );

    rot.Px = Px * R2D;
    rot.Py = Py * R2D;
    rot.Pz = Pz * R2D;

    return rot;
}


var update_jsbsim_joint = func( base ) {
    var fixed = {};
    fixed.Tx = getprop( base ~ "/roll" );
    fixed.Ty = getprop( base ~ "/pitch" );
    fixed.Tz = getprop( base ~ "/yaw" );

    var rot = rot_trans( fixed );

    setprop( base ~ "/phi-x", rot.Px );
    setprop( base ~ "/phi-y", rot.Py );
    setprop( base ~ "/phi-z", rot.Pz );
}


var update_jsbsim = func {
    update_jsbsim_joint("/fdm/jsbsim/Creare/left-forearm");
    update_jsbsim_joint("/fdm/jsbsim/Creare/right-forearm");
    update_jsbsim_joint("/fdm/jsbsim/Creare/left-lower-leg");    
    update_jsbsim_joint("/fdm/jsbsim/Creare/right-lower-leg");
    update_jsbsim_joint("/fdm/jsbsim/Creare/left-arm");
    update_jsbsim_joint("/fdm/jsbsim/Creare/right-arm");
    update_jsbsim_joint("/fdm/jsbsim/Creare/left-leg");
    update_jsbsim_joint("/fdm/jsbsim/Creare/right-leg");
    update_jsbsim_joint("/fdm/jsbsim/Creare/head");
    update_jsbsim_joint("/fdm/jsbsim/Creare/torso");
    update_jsbsim_joint("/fdm/jsbsim/Creare/pelvis");
}

var set_target_pose = func {
    var pose = poseNode.getValue();
    if ( pose == last_pose ) {
	return;
    } else {
	last_pose = pose;
    }

    if ( pose == "Neutral" ) {
    	setprop("/fdm/jsbsim/Creare/left-forearm/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/left-forearm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/left-forearm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-forearm/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/right-forearm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/right-forearm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-arm/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/left-arm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/left-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-arm/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/right-arm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/right-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/left-leg/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/left-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/right-leg/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/right-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/head/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/head/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/head/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/torso/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/torso/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/torso/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/pelvis/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/pelvis/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/pelvis/yaw-target", 0);
    } elsif ( pose == "Box" ) {
	setprop("/fdm/jsbsim/Creare/left-forearm/roll-target", 95);
	setprop("/fdm/jsbsim/Creare/left-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/left-forearm/yaw-target", -35);

	setprop("/fdm/jsbsim/Creare/right-forearm/roll-target", -95);
	setprop("/fdm/jsbsim/Creare/right-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/right-forearm/yaw-target", 35);

	setprop("/fdm/jsbsim/Creare/left-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", -40);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", -40);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-arm/roll-target", 80);
	setprop("/fdm/jsbsim/Creare/left-arm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/left-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-arm/roll-target", -80);
	setprop("/fdm/jsbsim/Creare/right-arm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/right-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-leg/roll-target", 20);
	setprop("/fdm/jsbsim/Creare/left-leg/pitch-target", -25);
	setprop("/fdm/jsbsim/Creare/left-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-leg/roll-target", -20);
	setprop("/fdm/jsbsim/Creare/right-leg/pitch-target", -25);
	setprop("/fdm/jsbsim/Creare/right-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/head/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/head/pitch-target", 40);
	setprop("/fdm/jsbsim/Creare/head/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/torso/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/torso/pitch-target", 15);
	setprop("/fdm/jsbsim/Creare/torso/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/pelvis/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/pelvis/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/pelvis/yaw-target", 0);
    } elsif ( pose == "Left Translation" ) {
	setprop("/fdm/jsbsim/Creare/left-forearm/roll-target", 95);
	setprop("/fdm/jsbsim/Creare/left-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/left-forearm/yaw-target", -35);

	setprop("/fdm/jsbsim/Creare/right-forearm/roll-target", -95);
	setprop("/fdm/jsbsim/Creare/right-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/right-forearm/yaw-target", 35);

	setprop("/fdm/jsbsim/Creare/left-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", -40);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", -40);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-arm/roll-target", 80);
	setprop("/fdm/jsbsim/Creare/left-arm/pitch-target", 15);
	setprop("/fdm/jsbsim/Creare/left-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-arm/roll-target", -80);
	setprop("/fdm/jsbsim/Creare/right-arm/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/right-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-leg/roll-target", 20);
	setprop("/fdm/jsbsim/Creare/left-leg/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/left-leg/yaw-target", -10);

	setprop("/fdm/jsbsim/Creare/right-leg/roll-target", -25);
	setprop("/fdm/jsbsim/Creare/right-leg/pitch-target", -25);
	setprop("/fdm/jsbsim/Creare/right-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/head/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/head/pitch-target", 40);
	setprop("/fdm/jsbsim/Creare/head/yaw-target", -10);

	setprop("/fdm/jsbsim/Creare/torso/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/torso/pitch-target", 15);
	setprop("/fdm/jsbsim/Creare/torso/yaw-target", 5);

	setprop("/fdm/jsbsim/Creare/pelvis/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/pelvis/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/pelvis/yaw-target", 5);
    } elsif ( pose == "Right Translation" ) {
	setprop("/fdm/jsbsim/Creare/left-forearm/roll-target", 95);
	setprop("/fdm/jsbsim/Creare/left-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/left-forearm/yaw-target", -35);

	setprop("/fdm/jsbsim/Creare/right-forearm/roll-target", -95);
	setprop("/fdm/jsbsim/Creare/right-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/right-forearm/yaw-target", 35);

	setprop("/fdm/jsbsim/Creare/left-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", -40);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", -40);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-arm/roll-target", 80);
	setprop("/fdm/jsbsim/Creare/left-arm/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/left-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-arm/roll-target", -80);
	setprop("/fdm/jsbsim/Creare/right-arm/pitch-target", 15);
	setprop("/fdm/jsbsim/Creare/right-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-leg/roll-target", 25);
	setprop("/fdm/jsbsim/Creare/left-leg/pitch-target", -25);
	setprop("/fdm/jsbsim/Creare/left-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-leg/roll-target", -20);
	setprop("/fdm/jsbsim/Creare/right-leg/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/right-leg/yaw-target", 10);

	setprop("/fdm/jsbsim/Creare/head/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/head/pitch-target", 40);
	setprop("/fdm/jsbsim/Creare/head/yaw-target", 10);

	setprop("/fdm/jsbsim/Creare/torso/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/torso/pitch-target", 15);
	setprop("/fdm/jsbsim/Creare/torso/yaw-target", -5);

	setprop("/fdm/jsbsim/Creare/pelvis/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/pelvis/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/pelvis/yaw-target", -5);
    } elsif ( pose == "Anterior Translation" ) {
	setprop("/fdm/jsbsim/Creare/left-forearm/roll-target", 105);
	setprop("/fdm/jsbsim/Creare/left-forearm/pitch-target", 35);
	setprop("/fdm/jsbsim/Creare/left-forearm/yaw-target", -35);

	setprop("/fdm/jsbsim/Creare/right-forearm/roll-target", -105);
	setprop("/fdm/jsbsim/Creare/right-forearm/pitch-target", 35);
	setprop("/fdm/jsbsim/Creare/right-forearm/yaw-target", 35);

	setprop("/fdm/jsbsim/Creare/left-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", -65);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", -65);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-arm/roll-target", 75);
	setprop("/fdm/jsbsim/Creare/left-arm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/left-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-arm/roll-target", -75);
	setprop("/fdm/jsbsim/Creare/right-arm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/right-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-leg/roll-target", 20);
	setprop("/fdm/jsbsim/Creare/left-leg/pitch-target", -12.5);
	setprop("/fdm/jsbsim/Creare/left-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-leg/roll-target", -20);
	setprop("/fdm/jsbsim/Creare/right-leg/pitch-target", -12.5);
	setprop("/fdm/jsbsim/Creare/right-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/head/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/head/pitch-target", 50);
	setprop("/fdm/jsbsim/Creare/head/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/torso/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/torso/pitch-target", 25);
	setprop("/fdm/jsbsim/Creare/torso/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/pelvis/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/pelvis/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/pelvis/yaw-target", 0);
    } elsif ( pose == "Posterior Translation" ) {
	setprop("/fdm/jsbsim/Creare/left-forearm/roll-target", 5);
	setprop("/fdm/jsbsim/Creare/left-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/left-forearm/yaw-target", -35);

	setprop("/fdm/jsbsim/Creare/right-forearm/roll-target", -5);
	setprop("/fdm/jsbsim/Creare/right-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/right-forearm/yaw-target", 35);

	setprop("/fdm/jsbsim/Creare/left-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", -40);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", -40);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-arm/roll-target", 155);
	setprop("/fdm/jsbsim/Creare/left-arm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/left-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-arm/roll-target", -155);
	setprop("/fdm/jsbsim/Creare/right-arm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/right-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-leg/roll-target", 20);
	setprop("/fdm/jsbsim/Creare/left-leg/pitch-target", -25);
	setprop("/fdm/jsbsim/Creare/left-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-leg/roll-target", -20);
	setprop("/fdm/jsbsim/Creare/right-leg/pitch-target", -25);
	setprop("/fdm/jsbsim/Creare/right-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/head/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/head/pitch-target", 20);
	setprop("/fdm/jsbsim/Creare/head/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/torso/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/torso/pitch-target", 10);
	setprop("/fdm/jsbsim/Creare/torso/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/pelvis/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/pelvis/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/pelvis/yaw-target", 0);
    } elsif ( pose == "Left Dorsoventral" ) {
	setprop("/fdm/jsbsim/Creare/left-forearm/roll-target", 95);
	setprop("/fdm/jsbsim/Creare/left-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/left-forearm/yaw-target", -35);

	setprop("/fdm/jsbsim/Creare/right-forearm/roll-target", -95);
	setprop("/fdm/jsbsim/Creare/right-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/right-forearm/yaw-target", 35);

	setprop("/fdm/jsbsim/Creare/left-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", -35);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", -50);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-arm/roll-target", 80);
	setprop("/fdm/jsbsim/Creare/left-arm/pitch-target", 15);
	setprop("/fdm/jsbsim/Creare/left-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-arm/roll-target", -80);
	setprop("/fdm/jsbsim/Creare/right-arm/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/right-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-leg/roll-target", 20);
	setprop("/fdm/jsbsim/Creare/left-leg/pitch-target", -25);
	setprop("/fdm/jsbsim/Creare/left-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-leg/roll-target", -35);
	setprop("/fdm/jsbsim/Creare/right-leg/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/right-leg/yaw-target", -25);

	setprop("/fdm/jsbsim/Creare/head/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/head/pitch-target", 40);
	setprop("/fdm/jsbsim/Creare/head/yaw-target", -10);

	setprop("/fdm/jsbsim/Creare/torso/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/torso/pitch-target", 15);
	setprop("/fdm/jsbsim/Creare/torso/yaw-target", 5);

	setprop("/fdm/jsbsim/Creare/pelvis/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/pelvis/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/pelvis/yaw-target", 0);
    } elsif ( pose == "Right Dorsoventral" ) {
	setprop("/fdm/jsbsim/Creare/left-forearm/roll-target", 95);
	setprop("/fdm/jsbsim/Creare/left-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/left-forearm/yaw-target", -35);

	setprop("/fdm/jsbsim/Creare/right-forearm/roll-target", -95);
	setprop("/fdm/jsbsim/Creare/right-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/right-forearm/yaw-target", 35);

	setprop("/fdm/jsbsim/Creare/left-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", -50);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", -35);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-arm/roll-target", 80);
	setprop("/fdm/jsbsim/Creare/left-arm/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/left-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-arm/roll-target", -80);
	setprop("/fdm/jsbsim/Creare/right-arm/pitch-target", 15);
	setprop("/fdm/jsbsim/Creare/right-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-leg/roll-target", 35);
	setprop("/fdm/jsbsim/Creare/left-leg/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/left-leg/yaw-target", -25);

	setprop("/fdm/jsbsim/Creare/right-leg/roll-target", -20);
	setprop("/fdm/jsbsim/Creare/right-leg/pitch-target", -25);
	setprop("/fdm/jsbsim/Creare/right-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/head/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/head/pitch-target", 40);
	setprop("/fdm/jsbsim/Creare/head/yaw-target", 10);

	setprop("/fdm/jsbsim/Creare/torso/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/torso/pitch-target", 15);
	setprop("/fdm/jsbsim/Creare/torso/yaw-target", -5);

	setprop("/fdm/jsbsim/Creare/pelvis/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/pelvis/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/pelvis/yaw-target", 0);
    } elsif ( pose == "Dorsal" ) {
	setprop("/fdm/jsbsim/Creare/left-forearm/roll-target", 15);
	setprop("/fdm/jsbsim/Creare/left-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/left-forearm/yaw-target", -35);

	setprop("/fdm/jsbsim/Creare/right-forearm/roll-target", -15);
	setprop("/fdm/jsbsim/Creare/right-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/right-forearm/yaw-target", 35);

	setprop("/fdm/jsbsim/Creare/left-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", -20);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", -20);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-arm/roll-target", 110);
	setprop("/fdm/jsbsim/Creare/left-arm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/left-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-arm/roll-target", -110);
	setprop("/fdm/jsbsim/Creare/right-arm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/right-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-leg/roll-target", 25);
	setprop("/fdm/jsbsim/Creare/left-leg/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/left-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-leg/roll-target", -25);
	setprop("/fdm/jsbsim/Creare/right-leg/pitch-target", -15);
	setprop("/fdm/jsbsim/Creare/right-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/head/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/head/pitch-target", 30);
	setprop("/fdm/jsbsim/Creare/head/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/torso/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/torso/pitch-target", 7.5);
	setprop("/fdm/jsbsim/Creare/torso/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/pelvis/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/pelvis/pitch-target", -7.5);
	setprop("/fdm/jsbsim/Creare/pelvis/yaw-target", 0);
    } elsif ( pose == "Ventral" ) {
	setprop("/fdm/jsbsim/Creare/left-forearm/roll-target", 105);
	setprop("/fdm/jsbsim/Creare/left-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/left-forearm/yaw-target", -35);

	setprop("/fdm/jsbsim/Creare/right-forearm/roll-target", -105);
	setprop("/fdm/jsbsim/Creare/right-forearm/pitch-target", -10);
	setprop("/fdm/jsbsim/Creare/right-forearm/yaw-target", 35);

	setprop("/fdm/jsbsim/Creare/left-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", -70);
	setprop("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-lower-leg/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", -70);
	setprop("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-arm/roll-target", 105);
	setprop("/fdm/jsbsim/Creare/left-arm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/left-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-arm/roll-target", -105);
	setprop("/fdm/jsbsim/Creare/right-arm/pitch-target", 0);
	setprop("/fdm/jsbsim/Creare/right-arm/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/left-leg/roll-target", 20);
	setprop("/fdm/jsbsim/Creare/left-leg/pitch-target", -25);
	setprop("/fdm/jsbsim/Creare/left-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/right-leg/roll-target", -20);
	setprop("/fdm/jsbsim/Creare/right-leg/pitch-target", -25);
	setprop("/fdm/jsbsim/Creare/right-leg/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/head/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/head/pitch-target", 40);
	setprop("/fdm/jsbsim/Creare/head/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/torso/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/torso/pitch-target", 17.5);
	setprop("/fdm/jsbsim/Creare/torso/yaw-target", 0);

	setprop("/fdm/jsbsim/Creare/pelvis/roll-target", 0);
	setprop("/fdm/jsbsim/Creare/pelvis/pitch-target", -17.5);
	setprop("/fdm/jsbsim/Creare/pelvis/yaw-target", 0);
    }

    update_rate("/fdm/jsbsim/Creare/left-forearm/roll-target", "/fdm/jsbsim/Creare/left-forearm/roll", "/fdm/jsbsim/Creare/left-forearm/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-forearm/pitch-target", "/fdm/jsbsim/Creare/left-forearm/pitch", "/fdm/jsbsim/Creare/left-forearm/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-forearm/yaw-target", "/fdm/jsbsim/Creare/left-forearm/yaw", "/fdm/jsbsim/Creare/left-forearm/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/right-forearm/roll-target", "/fdm/jsbsim/Creare/right-forearm/roll", "/fdm/jsbsim/Creare/right-forearm/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-forearm/pitch-target", "/fdm/jsbsim/Creare/right-forearm/pitch", "/fdm/jsbsim/Creare/right-forearm/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-forearm/yaw-target", "/fdm/jsbsim/Creare/right-forearm/yaw", "/fdm/jsbsim/Creare/right-forearm/yaw-rate", move_sec );
    
    update_rate("/fdm/jsbsim/Creare/left-lower-leg/roll-target", "/fdm/jsbsim/Creare/left-lower-leg/roll", "/fdm/jsbsim/Creare/left-lower-leg/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", "/fdm/jsbsim/Creare/left-lower-leg/pitch", "/fdm/jsbsim/Creare/left-lower-leg/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", "/fdm/jsbsim/Creare/left-lower-leg/yaw", "/fdm/jsbsim/Creare/left-lower-leg/yaw-rate", move_sec );
    
    update_rate("/fdm/jsbsim/Creare/right-lower-leg/roll-target", "/fdm/jsbsim/Creare/right-lower-leg/roll", "/fdm/jsbsim/Creare/right-lower-leg/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", "/fdm/jsbsim/Creare/right-lower-leg/pitch", "/fdm/jsbsim/Creare/right-lower-leg/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", "/fdm/jsbsim/Creare/right-lower-leg/yaw", "/fdm/jsbsim/Creare/right-lower-leg/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/left-arm/roll-target", "/fdm/jsbsim/Creare/left-arm/roll", "/fdm/jsbsim/Creare/left-arm/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-arm/pitch-target", "/fdm/jsbsim/Creare/left-arm/pitch", "/fdm/jsbsim/Creare/left-arm/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-arm/yaw-target", "/fdm/jsbsim/Creare/left-arm/yaw", "/fdm/jsbsim/Creare/left-arm/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/right-arm/roll-target", "/fdm/jsbsim/Creare/right-arm/roll", "/fdm/jsbsim/Creare/right-arm/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-arm/pitch-target", "/fdm/jsbsim/Creare/right-arm/pitch", "/fdm/jsbsim/Creare/right-arm/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-arm/yaw-target", "/fdm/jsbsim/Creare/right-arm/yaw", "/fdm/jsbsim/Creare/right-arm/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/left-leg/roll-target", "/fdm/jsbsim/Creare/left-leg/roll", "/fdm/jsbsim/Creare/left-leg/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-leg/pitch-target", "/fdm/jsbsim/Creare/left-leg/pitch", "/fdm/jsbsim/Creare/left-leg/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-leg/yaw-target", "/fdm/jsbsim/Creare/left-leg/yaw", "/fdm/jsbsim/Creare/left-leg/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/right-leg/roll-target", "/fdm/jsbsim/Creare/right-leg/roll", "/fdm/jsbsim/Creare/right-leg/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-leg/pitch-target", "/fdm/jsbsim/Creare/right-leg/pitch", "/fdm/jsbsim/Creare/right-leg/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-leg/yaw-target", "/fdm/jsbsim/Creare/right-leg/yaw", "/fdm/jsbsim/Creare/right-leg/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/head/roll-target", "/fdm/jsbsim/Creare/head/roll", "/fdm/jsbsim/Creare/head/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/head/pitch-target", "/fdm/jsbsim/Creare/head/pitch", "/fdm/jsbsim/Creare/head/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/head/yaw-target", "/fdm/jsbsim/Creare/head/yaw", "/fdm/jsbsim/Creare/head/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/torso/roll-target", "/fdm/jsbsim/Creare/torso/roll", "/fdm/jsbsim/Creare/torso/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/torso/pitch-target", "/fdm/jsbsim/Creare/torso/pitch", "/fdm/jsbsim/Creare/torso/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/torso/yaw-target", "/fdm/jsbsim/Creare/torso/yaw", "/fdm/jsbsim/Creare/torso/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/pelvis/roll-target", "/fdm/jsbsim/Creare/pelvis/roll", "/fdm/jsbsim/Creare/pelvis/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/pelvis/pitch-target", "/fdm/jsbsim/Creare/pelvis/pitch", "/fdm/jsbsim/Creare/pelvis/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/pelvis/yaw-target", "/fdm/jsbsim/Creare/pelvis/yaw", "/fdm/jsbsim/Creare/pelvis/yaw-rate", move_sec );
}


var update_rate = func( target_node, current_node, rate_node, eta_sec ) {
    var target_val = getprop( target_node );
    var current_val = getprop( current_node );
    if ( current_val == nil ) { current_val = 0.0; }
    if ( target_val == nil ) { target_val = 0.0; }
    var diff = abs( target_val - current_val );
    var rate = diff / eta_sec;		# deg per sec
    setprop( rate_node, rate );
}


var update_joint = func( target_node, current_node, rate_node, dt ) {
    var target_val = getprop( target_node );
    var current_val = getprop( current_node );
    var rate_val = getprop( rate_node ) * dt;
    if ( current_val == nil ) { current_val = 0.0; }
    if ( target_val == nil ) { target_val = 0.0; }
    if ( rate_val == nil ) { rate_val = 0.0; }
    var diff = target_val - current_val;
    if ( diff > rate_val ) { diff = rate_val; }
    if ( diff < -rate_val ) { diff = -rate_val; }
    current_val += diff;
    setprop( current_node, current_val );
}

var update_pose = func( dt ) {
    update_joint("/fdm/jsbsim/Creare/left-forearm/roll-target", "/fdm/jsbsim/Creare/left-forearm/roll", "/fdm/jsbsim/Creare/left-forearm/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-forearm/pitch-target", "/fdm/jsbsim/Creare/left-forearm/pitch", "/fdm/jsbsim/Creare/left-forearm/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-forearm/yaw-target", "/fdm/jsbsim/Creare/left-forearm/yaw", "/fdm/jsbsim/Creare/left-forearm/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/right-forearm/roll-target", "/fdm/jsbsim/Creare/right-forearm/roll", "/fdm/jsbsim/Creare/right-forearm/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-forearm/pitch-target", "/fdm/jsbsim/Creare/right-forearm/pitch", "/fdm/jsbsim/Creare/right-forearm/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-forearm/yaw-target", "/fdm/jsbsim/Creare/right-forearm/yaw", "/fdm/jsbsim/Creare/right-forearm/yaw-rate", dt );
    
    update_joint("/fdm/jsbsim/Creare/left-lower-leg/roll-target", "/fdm/jsbsim/Creare/left-lower-leg/roll", "/fdm/jsbsim/Creare/left-lower-leg/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", "/fdm/jsbsim/Creare/left-lower-leg/pitch", "/fdm/jsbsim/Creare/left-lower-leg/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", "/fdm/jsbsim/Creare/left-lower-leg/yaw", "/fdm/jsbsim/Creare/left-lower-leg/yaw-rate", dt );
    
    update_joint("/fdm/jsbsim/Creare/right-lower-leg/roll-target", "/fdm/jsbsim/Creare/right-lower-leg/roll", "/fdm/jsbsim/Creare/right-lower-leg/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", "/fdm/jsbsim/Creare/right-lower-leg/pitch", "/fdm/jsbsim/Creare/right-lower-leg/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", "/fdm/jsbsim/Creare/right-lower-leg/yaw", "/fdm/jsbsim/Creare/right-lower-leg/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/left-arm/roll-target", "/fdm/jsbsim/Creare/left-arm/roll", "/fdm/jsbsim/Creare/left-arm/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-arm/pitch-target", "/fdm/jsbsim/Creare/left-arm/pitch", "/fdm/jsbsim/Creare/left-arm/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-arm/yaw-target", "/fdm/jsbsim/Creare/left-arm/yaw", "/fdm/jsbsim/Creare/left-arm/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/right-arm/roll-target", "/fdm/jsbsim/Creare/right-arm/roll", "/fdm/jsbsim/Creare/right-arm/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-arm/pitch-target", "/fdm/jsbsim/Creare/right-arm/pitch", "/fdm/jsbsim/Creare/right-arm/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-arm/yaw-target", "/fdm/jsbsim/Creare/right-arm/yaw", "/fdm/jsbsim/Creare/right-arm/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/left-leg/roll-target", "/fdm/jsbsim/Creare/left-leg/roll", "/fdm/jsbsim/Creare/left-leg/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-leg/pitch-target", "/fdm/jsbsim/Creare/left-leg/pitch", "/fdm/jsbsim/Creare/left-leg/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-leg/yaw-target", "/fdm/jsbsim/Creare/left-leg/yaw", "/fdm/jsbsim/Creare/left-leg/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/right-leg/roll-target", "/fdm/jsbsim/Creare/right-leg/roll", "/fdm/jsbsim/Creare/right-leg/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-leg/pitch-target", "/fdm/jsbsim/Creare/right-leg/pitch", "/fdm/jsbsim/Creare/right-leg/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-leg/yaw-target", "/fdm/jsbsim/Creare/right-leg/yaw", "/fdm/jsbsim/Creare/right-leg/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/head/roll-target", "/fdm/jsbsim/Creare/head/roll", "/fdm/jsbsim/Creare/head/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/head/pitch-target", "/fdm/jsbsim/Creare/head/pitch", "/fdm/jsbsim/Creare/head/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/head/yaw-target", "/fdm/jsbsim/Creare/head/yaw", "/fdm/jsbsim/Creare/head/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/torso/roll-target", "/fdm/jsbsim/Creare/torso/roll", "/fdm/jsbsim/Creare/torso/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/torso/pitch-target", "/fdm/jsbsim/Creare/torso/pitch", "/fdm/jsbsim/Creare/torso/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/torso/yaw-target", "/fdm/jsbsim/Creare/torso/yaw", "/fdm/jsbsim/Creare/torso/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/pelvis/roll-target", "/fdm/jsbsim/Creare/pelvis/roll", "/fdm/jsbsim/Creare/pelvis/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/pelvis/pitch-target", "/fdm/jsbsim/Creare/pelvis/pitch", "/fdm/jsbsim/Creare/pelvis/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/pelvis/yaw-target", "/fdm/jsbsim/Creare/pelvis/yaw", "/fdm/jsbsim/Creare/pelvis/yaw-rate", dt );
}


var main_loop = func {
    var time = getprop("/sim/time/elapsed-sec");
    var dt = time - last_time;
    last_time = time;

    set_target_pose();
    update_pose( dt );
    update_jsbsim();

    var agl = getprop("/position/altitude-agl-ft");
    if ( agl <= end_simulation_agl and end_of_simulation == 0) {
        end_of_simulation = 1;
        setprop("/sim/freeze/master", 1);
        setprop("/sim/freeze/clock", 1);
        setprop("/sim/freeze/replay-state", 0);
    } elsif ( agl >= 1000 ) {
        end_of_simulation = 0;
    }

    settimer(main_loop, 0);
}

var setup_display = func {
     var display  = screen.display.new(20, -25);
     display.format = "%.1f";

     display.add("/velocities/vertical-speed-fps");
     display.add("/position/altitude-agl-ft");
     display.add("/velocities/uBody-fps");
     display.add("/velocities/vBody-fps");
     display.add("/orientation/roll-deg");
     display.add("/orientation/pitch-deg");
     display.add("/orientation/heading-deg");
     display.add("/orientation/roll-rate-degps");
     display.add("/orientation/pitch-rate-degps");
     display.add("/orientation/yaw-rate-degps");
}

setlistener("/sim/signals/fdm-initialized",
	    func {
		setup_display();
		main_loop();
	    });
