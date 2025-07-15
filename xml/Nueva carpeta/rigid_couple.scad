difference(){
cylinder(d=14,h=25,$fn=40);
translate([0,0,-10])cylinder(d=5,h=50,$fn=40);
translate([-5,0,7])rotate([0,90,0])cylinder(d=3,h=50,$fn=40);
translate([-5,0,18])rotate([0,90,0])cylinder(d=3,h=50,$fn=40);
}

