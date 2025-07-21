
/* -----------------------------------------------------------------------------
----------------------------------------------------------------------------- */


/*----------------------- Parameters Need By OpenScad ------------------------*/

$fn =30;                               // increase to 64 - 100 for final rendering

include <library/misc_parts.scad>;
include <library/misc_boards.scad>;
include <library/rpi-camera.scad>;
include <library/N20_Motor_Mounting_Bracket.scad>;



/*------------------ Hardware Defined Dimensional Parameter ------------------*/

rpi5_length = 85;          // in mm (3.35 inches)
rpi5_width = 56;           // in mm (2.20 inches)
rpi5_height = 17;          // in mm (0.67 inches)

/*------------------- Target Overall Dimensional Parameters ------------------*/

// base plate
main_plate_x = 100;
main_plate_y = 60;
main_plate_z = 3;

// motor plate
motor_plate_x = 50;
motor_plate_y = 30;
motor_plate_z = main_plate_z;

// support cylinders
lenght = 160;
radius = 1;

/*---------------- Derived & Specified Dimensional Parameters ----------------*/




/*----------------------------- Support Modules ------------------------------*/

module main_body(thickness) {
    //cube(size = [main_plate_x, main_plate_y, thickness], center = true);
    thk = thickness / 2;

    hull() {
        translate([main_plate_x / 2, main_plate_y / 2, 0]) sphere(thk);
        translate([-main_plate_x / 2, -main_plate_y / 2, 0]) sphere(thk);
        translate([-main_plate_x / 2, main_plate_y / 2, 0]) sphere(thk);
        translate([main_plate_x / 2, -main_plate_y / 2, 0]) sphere(thk);
    }
}


module main_body_cutter(thickness) {
    cube(size = [main_plate_x, main_plate_y, thickness], center = true);
}


module motor_body(zr, xt, thickness) {
    thk = thickness / 2;

    difference() {
        rotate([0, 0, zr]) translate([xt, 0, 0]) hull() {
            //cube(size = [motor_plate_x, motor_plate_y, thickness], center = true);
            translate([motor_plate_x / 2, motor_plate_y / 2, 0]) sphere(thk);
            translate([-motor_plate_x / 2, -motor_plate_y / 2, 0]) sphere(thk);
            translate([-motor_plate_x / 2, motor_plate_y / 2, 0]) sphere(thk);
            translate([motor_plate_x / 2, -motor_plate_y / 2, 0]) sphere(thk);
        }
    
    // drill holes in motor support
    rotate([0, 0, zr]) translate([xt - 20,-9.25,0]) cylinder(h = 6, r = 1, center = true);
    rotate([0, 0, zr]) translate([xt - 20,9.25,0])cylinder(h = 6, r = 1, center = true);
    }
}


module motor_body_cutter(zr, xt, thickness) {
    rotate([0, 0, zr]) translate([xt, 0, 0]) cube(size = [motor_plate_x, motor_plate_y, thickness], center = true);
}


module whole_body(thickness) {
    main_body(thickness);
    motor_body(0, -50, thickness);
    motor_body(120, -50, thickness);
    motor_body(-120, -50, thickness);
}


module whole_body_cutter(thickness) {
    main_body_cutter(thickness);
    motor_body_cutter(0, -50, thickness);
    motor_body_cutter(120, -50, thickness);
    motor_body_cutter(-120, -50, thickness);
}


module one_bracket() {
    N20_Motor_Mounting_Bracket();
}


/*------------------------------ Main Modules --------------------------------*/

module body() {
    whole_body(main_plate_z);                      // laydown the body

    intersection() {                               // laydown the suports for the body
        whole_body_cutter(3 * main_plate_z);
        union() {
          support_pattern(0, 0, 45, radius);
          support_pattern(0, 0, -45, radius);
        }
    }  
}


module support_pattern(x, y, z, rad) {
    rotate([x,y,z]) {
        translate([0, -60, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = rad, center = true);
        translate([0, -40, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = rad, center = true);
        translate([0, -20, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = rad, center = true);

        translate([0, 0, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = rad, center = true);      // center support

        translate([0, 20, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = rad, center = true);
        translate([0, 40, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = rad, center = true);
        translate([0, 60, 2]) rotate([0, 90, 0]) cylinder(h = lenght, r = rad, center = true);
    } 
}


module all_brackets() {
    translate([-10, -50, 0]) one_bracket();
    translate([-40, -50, 0]) one_bracket();
    translate([-70, -50, 0]) one_bracket();
}


/*
module RPi_5() {
    translate([-rpi5_length / 2, -rpi5_width / 2, -10]) rotate([-90, 0, 0]) import("library/raspberry_pi_5.stl");
}
*/
module RPi_5() {
    translate([-rpi5_length / 2, -rpi5_width / 2, -10]) rotate([0, 0, -90]) rotate([0, -180, 0]) board_raspberrypi_4_model_b();
}


module camera() {
   translate([0, 0, -40]) rpi_camera();
}



/*------------------------------ Main Execution ------------------------------*/

// laydown the body
color("DarkKhaki") body();

// laydown the motor brackets
color("DarkKhaki") all_brackets();

RPi_5();
camera();
