
var presets = func{

    var speed = getprop("/environment/wind-speed-kt", 0);
    var heading = getprop("/environment/wind-from-heading-deg", 0);

    if (speed) setprop("sim/presets/offset-distance-nm", speed);
    if (heading) setprop("sim/presets/offset-azimuth-deg", heading);

    if (!speed and !heading) settimer(presets, 0.1);
}

presets();
