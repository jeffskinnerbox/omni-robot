

/*----------------------- Parameters Need By OpenScad ------------------------*/
$fn =30;                        // increase to 64 - 100 for final rendering


// These lines set the resolution for the rendering. $fa is the minimum angle (in degrees) and $fs is the minimum size of a segment. Lower values mean higher resolution but slower rendering.
$fa = 1;                        // set the minimum angle for rendering
$fs = 0.4;                      // set the minimum size for rendering


/*--------------------- STL Objects Defined and Imported ---------------------*/


/*------------------ OpenScad Objects Defined and Included -------------------*/
//include <library/misc_parts.scad>;
//include <library/misc_boards.scad>;
//include <library/rpi-camera.scad>;
//include <library/N20_Motor_Mounting_Bracket.scad>;


/*------------------ Hardware Defined Dimensional Parameter ------------------*/

// https://cdn-shop.adafruit.com/product-files/4638/n20+motors_C15008.pdf
N20_length = 9+22-5;            // in mm (X inches), assuming sensor is 5 mm in length
N20_width = 12;                 // in mm (X inches)
N20_height = 10;                // in mm (X inches)
N20_sensor_length = 5;          // in mm (X inches)
N20_sensor_width = 12;          // in mm (X inches)
N20_sensor_height = 17.5;       // in mm (X inches)
N20_shaft_length = 10;          // in mm (X inches)
N20_shaft_diameter = 3.25;      // in mm (X inches)

// https://www.amazon.com/dp/B0DYDW239W
wheel_radius = 48/2;            // in mm (0.95 inches) radius of the circle's that forms the tube
wheel_width = 20;               // diameter of the circle that forms the tube
//wheel_width = 25;  change this Wheel Width: 25mm / 1.0in
//wheel_rolling_diameter = 13;  Roll Diameter: 13mm / 0.5in



/*------------------- Overall Target Dimensional Parameters ------------------*/


/*---------------- Derived & Specified Dimensional Parameters ----------------*/


/*------------------------------ Main Execution ------------------------------*/

rotate([0, 0, 0]) WheelAssembly();


/*------------------------------ Main Modules --------------------------------*/


/*----------------------------- Support Modules ------------------------------*/

module WheelAssembly() {
  translate([0, -22 -N20_length, N20_height/2 + 5]) {
    translate([0, 120, -N20_height/2]) Motor();
    translate([0, 125, -N20_height/2]) Bracket();
    translate([0, 134, 0]) WheelHub();
    translate([0, 154, 0]) Wheel();
  }
}


module Motor() {
  translate([0, 0, N20_height/2]) rotate([0, 90, -90]) union() {
    color("Gray") cube([N20_height, N20_width, N20_length], center = true);
    color("Brown") translate([(N20_height - N20_sensor_height)/2, 0, N20_length/2 - N20_sensor_length/2]) cube([N20_sensor_height, N20_sensor_width, N20_sensor_length], center = true);
    color("Gold") translate([0, 0, -N20_length/2 - N20_shaft_length/2]) cylinder(h = N20_shaft_length, r = N20_shaft_diameter / 2, center = true);
  }
}


module Bracket() {
    rotate([90, 0, 0]) translate([45, -90.6, -6.5]) import("/home/jeff/src/projects/omnidirectional-robot/omni-robot/cad/library/N20_Motor_Mounting_Bracket.stl");
}


module WheelHub() {
   color("Orange") translate([0, 4.6, 0]) rotate([0, 0, -120]) translate([50, 78, 0]) import("library/mini-loki-tire-hub.stl");
}


module Wheel() {
  color("Black") rotate([90, 0, 0]) rotate_extrude(angle = 360) {
    translate([wheel_radius, 0]) {
      circle(d = wheel_width);
    }
  }
}

