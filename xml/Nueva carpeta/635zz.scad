difference(){
union(){
difference(){
  cylinder(d=19,h=6,$fn=60);
  translate([0,0,5])cylinder(d=19-2,h=2,$fn=30);
  translate([0,0,0])cylinder(d=19-2,h=1,$fn=30);
  
}
cylinder(d=7,h=6,$fn=30);
}
  translate([0,0,-10])cylinder(d=5,h=100,$fn=30);
}