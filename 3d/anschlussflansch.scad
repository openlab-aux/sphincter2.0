

module grundplatte (X,Y,Z,r)
{
    difference()
    {
        hull()
        {
            translate([X/2-r,Y/2-r,0])cylinder(r=r,h=Z);
            translate([-X/2+r,Y/2-r,0])cylinder(r=r,h=Z);
            translate([X/2-r,-Y/2+r,0])cylinder(r=r,h=Z);
            translate([-X/2+r,-Y/2+r,0])cylinder(r=r,h=Z);
        }
        
        translate([X/2-r,Y/2-r,0])cylinder(d=3.6,h=Z);
        translate([-X/2+r,Y/2-r,0])cylinder(d=3.6,h=Z);
        translate([X/2-r,-Y/2+r,0])cylinder(d=3.6,h=Z);
        translate([-X/2+r,-Y/2+r,0])cylinder(d=3.6,h=Z);
        translate([0,-Y/2+r,0])cylinder(d=3.6,h=Z);
        translate([0,Y/2-r,0])cylinder(d=3.6,h=Z);
    }
}

module optointerface ()
{
    translate([8+0.7,0,0])cube([16,9.5,10],center=true);
    translate([-8-0.7,0,0])cube([16,9.5,10],center=true);
}
module optoinferface_befestigung ()
{
    difference(){
        union(){
            //translate([0.63+16+2.54,1.27,0])cylinder(d=5,h=6.5,$fn=20);
            //translate([-(0.63+16+2.54),1.27,0])cylinder(d=5,h=6.5,$fn=20);
            
            translate([16.8,-9.5/2,0])cube([6,9.5,6.5]);
            translate([-16.8-5,-9.5/2,0])cube([5,9.5,6.5]);
        }
        translate([0.63+16+2.54,1.27,0])cylinder(d=2.9,h=7,$fn=20);
        translate([-(0.63+16+2.54),1.27,0])cylinder(d=2.9,h=7,$fn=20);
    }
}

module sub_d ()
{
    cube([20.5,13,10],center=true);
    translate([12.5,0,0])cylinder(d=3.5,h=10,$fn=20);
    translate([-12.5,0,0])cylinder(d=3.5,h=10,$fn=20);  
}

difference(){
    
    grundplatte(110,35,2.5,5);
    

    
    translate([40,0,-1])cylinder(d=9.8,h=10);
    translate([13,0,-1])sub_d();
    translate([-27,0,-1])optointerface();
}
translate([-27,0,2.5])optoinferface_befestigung();