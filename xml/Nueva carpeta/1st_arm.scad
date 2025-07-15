a=100;//length
b=20;
ht=16;
translate([0, 0, ht+0.5])import("40pulley.stl");

difference(){
union(){



translate([a, 0, 0])cylinder(d=24,h=8.5,$fn=100);
translate([0, 0, 0])cylinder(d=28,h=ht,$fn=100);
translate([0, -b/2, 0])cube([100,b,5]);
    hull(){
translate([1, b-12, 3])cube([1,4,1],center=true);
translate([a-5, b-12, 5])cube([1,4,7],center=true);
}
hull(){
translate([1, b-28, 3])cube([1,4,1],center=true);
translate([a-5, b-28, 5])cube([1,4,7],center=true);
}


}
translate([0, 0, -10])cylinder(d=19,h=100,$fn=100);
hull(){
translate([25, 0, -10])cylinder(d=5,h=20,$fn=20);
translate([a-25, 0, -10])cylinder(d=5,h=20,$fn=20);
}
translate([a, 0, -10])cylinder(d=5.2,h=20,$fn=30); // drill for bearing 2d arm
translate([-10, 0, 0])cube([10,6,ht*2],center=true);

translate([a, 0, 7])cylinder(d=21,h=3,$fn=100);
}

