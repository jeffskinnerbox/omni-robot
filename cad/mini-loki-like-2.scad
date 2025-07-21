
/* -----------------------------------------------------------------------------

Design ideas taken from Mini Loki - Omnidirectional robotic platform - https://cults3d.com/en/3d-model/gadget/mini-loki-omnidirectional-robotic-platform 

----------------------------------------------------------------------------- */


/*----------------------- Parameters Need By OpenScad ------------------------*/
$fn =30;                   // increase to 64 - 100 for final rendering


// These lines set the resolution for the rendering. $fa is the minimum angle (in degrees) and $fs is the minimum size of a segment. Lower values mean higher resolution but slower rendering.
$fa = 1; // set the minimum angle for rendering
$fs = 0.4; // set the minimum size for rendering


/*--------------------- STL Objects Defined and Imported ---------------------*/


/*------------------ OpenScad Objects Defined and Included -------------------*/
//include <library/misc_parts.scad>;
include <library/misc_boards.scad>;
include <library/rpi-camera.scad>;
//include <library/N20_Motor_Mounting_Bracket.scad>;


/*------------------ Hardware Defined Dimensional Parameter ------------------*/
// https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-product-brief.pdf
// https://forums.raspberrypi.com/viewtopic.php?t=6934
rpi5_length = 85;                      // in mm (3.35 inches), does not account for the slightly overlapping SD card
rpi5_width = 56;                       // in mm (2.20 inches)
rpi5_height = 21.2;                    // in mm (X inches),

// https://cdn-shop.adafruit.com/product-files/4638/n20+motors_C15008.pdf
N20_length = 9+22-5;                   // in mm (X inches), this assuming sensor is 5 mm in length
N20_width = 12;                        // in mm (X inches)
N20_height = 10;                       // in mm (X inches)

N20_sensor_length = 5;                 // in mm (X inches)
N20_sensor_width = 12;                 // in mm (X inches)
N20_sensor_height = 17.5;              // in mm (X inches)

N20_shaft_length = 10;                 // in mm (X inches)
N20_shaft_diameter = 3;                // in mm (X inches)

// https://www.amazon.com/dp/B0DYDW239W
wheel_radius = 48/2;                   // in mm radius of the circle's that forms the tube
tire_diameter = 20;                    // in mm diameter of the circle that forms the tube
//Wheel Diameter: 48mm / 1.9in
//Wheel Width: 25mm / 1.0in
//Roll Diameter: 13mm / 0.5in


/*------------------- Overall Target Dimensional Parameters ------------------*/
fatness = 10;                             // additional space given to the body diameter to fit more than just the raspberry pi
body_diameter = rpi5_length + fatness;    // ???
body_height = rpi5_height * 2;            // ???

triangle_side = rpi5_length * 3.5;        // ???

rounding = 3;                             // minkowski sum rounding shpere in mm
lid_thinkness = rounding * 2;             // thickness of the lid that will be cut from the body


/*---------------- Derived & Specified Dimensional Parameters ----------------*/
cut_offset = body_diameter * 1.5;
cut_height = body_height * 2.5;


/*------------------------------ Main Execution ------------------------------*/

//SolidBody();
//RoundedBody();
//HollowBody();
//BottomBody();
BottomBody();
translate([0, 0, 60]) TopBody();

rotate([0, 0, 0]) WheelAssembly();
rotate([0, 0, 120]) WheelAssembly();
rotate([0, 0, 240]) WheelAssembly();

rotate([0, 0, 180]) translate([0, 0, 3 + 3]) RPi_5();

// TBD - RPi5 Mounting Points
// TBD - Motor, Camera Cuts
// TBD - Charging, USB, and Ethernet Cuts
// TBD - Battery and Charger
// TBD - IMU
// ------- Future -------
// TBD - LIDAR


/*----------------------------- Support Modules ------------------------------*/

module SolidBody() {
  difference() {
    color("Blue") rotate([0, 0, 60]) EquilateralTriangle(triangle_side, body_height);
    color("Red") rotate([0, 0, 0]) translate([0, -cut_offset, 0]) cylinder(h = cut_height, r = body_diameter, center = true);
    color("Red") rotate([0, 0, 120]) translate([0, -cut_offset, 0]) cylinder(h = cut_height, r = body_diameter, center = true);
    color("Red") rotate([0, 0, 240]) translate([0, -cut_offset, 0]) cylinder(h = cut_height, r = body_diameter, center = true);
  }
}


module RoundedBody() {
  minkowski() {
    SolidBody();
    sphere(r = rounding, $fn = 64);     // the minkowski sum will add thinkness of "rounding" to the ouside of the object
  }
}


module HollowBody() {
  difference() {
    RoundedBody();
     SolidBody();
  }
}


module BottomBody() {
  side = triangle_side;
  cut_position = body_height - lid_thinkness;

  translate([0, 0, rounding]) difference() {      // translate up to compensate for added thinkness by the minkowski sum
    HollowBody();
    translate([-side/2, -side/2, cut_position]) cube([side, side, body_height], center = false);
  }

  color("Red") translate([25, 89, body_height * 3/4]) Branding();
}


module TopBody() {
  side = triangle_side;

  cut_position = body_height - lid_thinkness;

  translate([0, 0, rounding]) intersection() {      // translate up to compensate for added thinkness by the minkowski sum
    HollowBody();
    translate([-side/2, -side/2, cut_position]) cube([side, side, body_height], center = false);
  }

  translate([0, 80, body_height + lid_thinkness]) Camera();

  color("Red") translate([25, 60, 48.5]) rotate([90, 0, 0]) Branding();
}


/*------------------------------ Main Modules --------------------------------*/

module EquilateralTriangle(side_length, z_height) {
  triangle_height = side_length * sqrt(3) / 2;             // calculate the height of the equilateral triangle

  // Calculate the coordinates of the vertices such that the centroid is at the origin
  // Vertex 1: (0, 2/3 * height)
  // Vertex 2: (-side_length/2, -1/3 * height)
  // Vertex 3: (side_length/2, -1/3 * height)
  point1 = [0, 2/3 * triangle_height];
  point2 = [-side_length/2, -1/3 * triangle_height];
  point3 = [side_length/2, -1/3 * triangle_height];

  linear_extrude(height = z_height, center = false) {
    polygon(points = [point1, point2, point3]);
  }

}


module WheelAssembly() {
  translate([0, -22 -N20_length, N20_height/2 + 5]) {
    translate([0, 120, -N20_height/2]) Motor();
    translate([0, 125, -N20_height/2]) MotorBracket();
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


module MotorBracket() {
    rotate([90, 0, 0]) translate([45, -90.6, -6.5]) import("library/N20_Motor_Mounting_Bracket.stl");
}


module WheelHub() {
   color("Orange") translate([0, 4.6, 0]) rotate([0, 0, -120]) translate([50, 78, 0]) import("library/mini-loki-tire-hub.stl");
}


module Wheel() {
  color("Black") rotate([90, 0, 0]) rotate_extrude(angle = 360) {
    translate([wheel_radius, 0]) {
      circle(d = tire_diameter);
    }
  }
}


module RPi_5() {
    translate([-rpi5_width / 2, -rpi5_length / 2, 0]) board_raspberrypi_4_model_b();
}


module Camera() {
  translate([0, -15, 0]) rotate([90, 0, 180]) translate([-28/2, 0, 0]) import("library/pi-camera-v3-bracket.stl");
   translate([12.5, 0, 24 + 9]) rotate([0, 90, 90]) rpi_camera();
}


module Branding() {
  rotate([-90, 180, 0]) text("Mini-Loki-Like", size = 6);
}

