var angle_transform = func( fixed ) {
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

var fixed = {};
var rot = {};
fixed.Tx = 20;  fixed.Ty = 40;  fixed.Tz = 60;  rot = angle_transform( fixed ); print("Px = ", rot.Px, " Py = ", rot.Py, " Pz = ", rot.Pz);
fixed.Tx = 60;  fixed.Ty = 10;  fixed.Tz = 70;  rot = angle_transform( fixed ); print("Px = ", rot.Px, " Py = ", rot.Py, " Pz = ", rot.Pz);
fixed.Tx = 90;  fixed.Ty = -60; fixed.Tz = -20; rot = angle_transform( fixed ); print("Px = ", rot.Px, " Py = ", rot.Py, " Pz = ", rot.Pz);
fixed.Tx = 120; fixed.Ty = -30; fixed.Tz = 60;  rot = angle_transform( fixed ); print("Px = ", rot.Px, " Py = ", rot.Py, " Pz = ", rot.Pz);
fixed.Tx = -45; fixed.Ty = 45;  fixed.Tz = -45; rot = angle_transform( fixed ); print("Px = ", rot.Px, " Py = ", rot.Py, " Pz = ", rot.Pz);
fixed.Tx = -20; fixed.Ty = 75;  fixed.Tz = 30;  rot = angle_transform( fixed ); print("Px = ", rot.Px, " Py = ", rot.Py, " Pz = ", rot.Pz);
