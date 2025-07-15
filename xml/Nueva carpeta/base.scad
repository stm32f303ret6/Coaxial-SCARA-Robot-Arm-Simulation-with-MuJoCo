include <Nema17.scad>;
a=42.3;
b=5.0;
c=31.0;
f=22.6;
hole=3.2;
res=16;


Nema17();
translate([a+0.1, 0, 0])Nema17();

color([0,0.8,0])
difference(){
translate([a/2, 0, b/2-2])cube([a*2, a, b], center=true);

translate([c/2, c/2, -50])cylinder(d=hole, h=100, $fn=res);
translate([-c/2, c/2, -50])cylinder(d=hole, h=100, $fn=res);
translate([c/2, -c/2, -50])cylinder(d=hole, h=100, $fn=res);
translate([-c/2, -c/2, -50])cylinder(d=hole, h=100, $fn=res);

translate([a+c/2, c/2, -50])cylinder(d=hole, h=100, $fn=res);
translate([a+-c/2, c/2, -50])cylinder(d=hole, h=100, $fn=res);
translate([a+c/2, -c/2, -50])cylinder(d=hole, h=100, $fn=res);
translate([a+-c/2, -c/2, -50])cylinder(d=hole, h=100, $fn=res);

translate([0.0, 0.0, -5])cylinder(d=f, h=100, $fn=50);
translate([a, 0.0, -5])cylinder(d=f, h=100, $fn=50);
    
translate([c/2, c/2, 0])cylinder(d=hole+3, h=10, $fn=res);
translate([-c/2, c/2, 0])cylinder(d=hole+3, h=100, $fn=res);
translate([c/2, -c/2, 0])cylinder(d=hole+3, h=100, $fn=res);
translate([-c/2, -c/2, 0])cylinder(d=hole+3, h=100, $fn=res);

translate([a+c/2, c/2, 0])cylinder(d=hole+3, h=100, $fn=res);
translate([a+-c/2, c/2, 0])cylinder(d=hole+3, h=100, $fn=res);
translate([a+c/2, -c/2, 0])cylinder(d=hole+3, h=100, $fn=res);
translate([a+-c/2, -c/2, 0])cylinder(d=hole+3, h=100, $fn=res);
}


color([0.6,0.6,00.4])translate([a, 0, 4])import("635zz.stl");
color([0.8,0,0])translate([0, 0, 20])rotate([0,180,0])import("pulley_1st_motor.stl");
color([0.6,0.6,0.6])translate([a, 0, 30])rotate([0,180,180])import("1st_arm.stl");
color([0.8,0,0])translate([a, 0, 51])rotate([0,180,180])import("pulley_1st_motor.stl");
color([0.2,0.2,0.2])translate([a, 0, 10])rotate([0,0,180])import("rigid_couple.stl");

color([0.6,0.6,00.4])translate([a+100, 0, 35])import("635zz.stl");
color([0.6,0.6,0.6])translate([a+100, 0, 48])rotate([0,180,180])import("2d_arm.stl");