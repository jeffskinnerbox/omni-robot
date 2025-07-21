

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
rpi5_length = 85;          // in mm (3.35 inches)
rpi5_width = 56;           // in mm (2.20 inches)
rpi5_height = 17;          // in mm (0.67 inches)

// https://cdn-shop.adafruit.com/product-files/4638/n20+motors_C15008.pdf
N20_length = 9+22-5;       // in mm (X inches), assuming sensor is 5 mm in length
N20_width = 12;            // in mm (X inches)
N20_height = 10;           // in mm (X inches)
N20_sensor_length = 5;     // in mm (X inches)
N20_sensor_width = 12;     // in mm (X inches)
N20_sensor_height = 17.5;  // in mm (X inches)
N20_shaft_length = 10;     // in mm (X inches)
N20_shaft_diameter = 3.25; // in mm (X inches)

wheel_radius = 48/2;       // radius of the circle's that forms the tube
tire_diameter = 20;        // diameter of the circle that forms the tube


/*------------------- Overall Target Dimensional Parameters ------------------*/
body_radius = rpi5_length + 10;
body_height = rpi5_height * 2;


/*---------------- Derived & Specified Dimensional Parameters ----------------*/
cut_offset = body_radius * 1.6;
cut_height = body_height * 2.5;


/*------------------------------ Main Execution ------------------------------*/

Body();

rotate([0, 0, 180]) translate([0, 0, 3 + 3]) RPi_5();

translate([0, 80, body_height + 7]) Camera();


/*------------------------------ Main Modules --------------------------------*/

module Body() {
  difference() {
    translate([0, 0, 0]) RoundedBody(3);
    translate([0, 0, 3]) RawBody();
  }
}

module RoundedBody(radius) {
  minkowski() {
    RawBody();
    sphere(r = radius, $fn = 64);
  }
}

module RawBody() {
  difference() {
    color("Blue") rotate([0, 0, 60]) EquilateralTriangle(rpi5_length * 3.5, body_height);
    color("Red") rotate([0, 0, 0]) translate([0, -cut_offset, 0]) cylinder(h = cut_height, r = body_radius, center = true);
    color("Red") rotate([0, 0, 120]) translate([0, -cut_offset, 0]) cylinder(h = cut_height, r = body_radius, center = true);
    color("Red") rotate([0, 0, 240]) translate([0, -cut_offset, 0]) cylinder(h = cut_height, r = body_radius, center = true);
  }
}


module RPi_5() {
    translate([-rpi5_width / 2, -rpi5_length / 2, 0]) board_raspberrypi_4_model_b();
}


module Camera() {
   translate([12.5, 0, 24]) rotate([0, 90, 90]) rpi_camera();
}


/*----------------------------- Support Modules ------------------------------*/

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


