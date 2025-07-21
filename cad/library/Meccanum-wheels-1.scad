$fn = (30);

// base plate
base_plate_x = 100;
base_plate_y = 60;
base_plate_z = 3;

// motor plate
motor_plate_x = 50;
motor_plate_y = 30;
motor_plate_z = base_plate_z;

// support cylinders
lenght = 160;
radius = 2;



module body(thickness) {
    cube(size = [base_plate_x, base_plate_y, thickness], center = true);
    motor_support(0, -50, thickness);
    motor_support(120, -50, thickness);
    motor_support(-120, -50, thickness);
}

module motor_support(zr, xt, thick) {
    difference() {
        rotate([0, 0, zr]) translate([xt, 0, 0]) cube(size = [motor_plate_x, motor_plate_y, thick], center = true);
    
    // drill holes in motor support
    rotate([0, 0, zr]) translate([xt - 20,-9.25,0]) cylinder(h = 6, r1 = 1, r2 = 1, center = true);
    rotate([0, 0, zr]) translate([xt - 20,9.25,0])cylinder(h = 6, r1 = 1, r2 = 1, center = true);
    }
    
    //color("Green") rotate([0, 0, zr]) translate([xt-20, 0, -0.5]) bracket();
}

module support_pattern(x, y, z) {
    rotate([x,y,z]) {
        translate([0, -60, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = radius, center = true);
        translate([0, -40, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = radius, center = true);
        translate([0, -20, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = radius, center = true);
        translate([0, 0, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = radius, center = true);
        translate([0, 20, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = radius, center = true);
        translate([0, 40, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = radius, center = true);
        translate([0, 60, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = radius, center = true);
    } 
}


module bracket() {
    translate([0, -50, 0]) rotate([90, 180, 90]) translate([45, -90.6, -6.5]) import("/home/jeff/src/projects/omnidirectional-robot/omni-robot/cad/stl-gcode/N20_Motor_Mounting_Bracket.stl");
}

/*

color("Red") body(base_plate_z);  
intersection() {
    body(3 * base_plate_z);
    union() {
        support_pattern(0, 0, 45);
        support_pattern(0, 0, -45);
    }
}  
*/
rotate([0, 180, 0]) translate([0, 50, 0]) bracket();







/*
cube(size = [100, 60, 5], center = true);

rotate ([90,0,90]) {
    translate ([90,-93,-15])
    import("/home/jeff/src/projects/omnidirectional-robot/cad/N20_Motor_Mounting_Bracket.stl");
    
    translate ([90,-93,-30])
    import("/home/jeff/src/projects/omnidirectional-robot/cad/N20_Motor_Mounting_Bracket.stl");

    translate ([90,-93,-45])
    import("/home/jeff/src/projects/omnidirectional-robot/cad/N20_Motor_Mounting_Bracket.stl");
}
 translate ([10,0,0]) {
difference () {
    
translate([-60,0,0])
cube(size = [40, 30, 5], center = true);

translate([-70,-9.25,0])
cylinder(h = 6, r1 = 1, r2 = 1, center = true);

translate([-70,9.25,0])
cylinder(h = 6, r1 = 1, r2 = 1, center = true);
}
}
rotate ([0,0,120]) {
 translate ([10,0,0]) {
difference () {
    
translate([-55,0,0])
cube(size = [50, 30, 5], center = true);

translate([-70,-9.25,0])
cylinder(h = 6, r1 = 1, r2 = 1, center = true);

translate([-70,9.25,0])
cylinder(h = 6, r1 = 1, r2 = 1, center = true);
}
}
}
rotate ([0,0,240]) {
 translate ([10,0,0]) {
difference () {
    
translate([-55,0,0])
cube(size = [50, 30, 5], center = true);

translate([-70,-9.25,0])
cylinder(h = 6, r1 = 1, r2 = 1, center = true);

translate([-70,9.25,0])
cylinder(h = 6, r1 = 1, r2 = 1, center = true);
}
}
}
rotate ([-90,0,119]) {
difference () {
translate ([0,0,0])
cylinder(h = 110, r1 = 4, r2 = 4, center = true);

translate ([0,4,0])
cube(size = [8, 8, 117], center = true);
}
}

translate ([24,-15,0])
rotate ([-90,0,119]) {
difference () {
translate ([0,0,0])
cylinder(h = 55, r1 = 4, r2 = 4, center = true);

translate ([0,4,0])
cube(size = [8, 8, 117], center = true);
}
}

translate ([-21,14,0])
rotate ([-90,0,119]) {
difference () {
translate ([0,0,0])
cylinder(h = 55, r1 = 4, r2 = 4, center = true);

translate ([0,4,0])
cube(size = [8, 8, 117], center = true);
}
}


rotate ([-90,0,241.1]) {
difference () {
translate ([0,0,0])
cylinder(h = 110, r1 = 4, r2 = 4, center = true);

translate ([0,4,0])
cube(size = [8, 8, 117], center = true);
}
}
translate ([24,15,0])
rotate ([-90,0,241.1]) {
difference () {
translate ([0,0,0])
cylinder(h = 55, r1 = 4, r2 = 4, center = true);

translate ([0,4,0])
cube(size = [8, 8, 117], center = true);
}
}
translate ([-21,-14,0])
rotate ([-90,0,241.1]) {
difference () {
translate ([0,0,0])
cylinder(h = 55, r1 = 4, r2 = 4, center = true);

translate ([0,4,0])
cube(size = [8, 8, 117], center = true);
}
}
*/
