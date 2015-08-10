L = 60;         //Länge
B = 60;         //Breite
H = 25;         //Höhe
D = 2;        //Wanddicke

module sub_d ()
{
    cube([20.5,13,10],center=true);
    translate([12.5,0,0])cylinder(d=3,h=10,$fn=20,center=true);
    translate([-12.5,0,0])cylinder(d=3,h=10,$fn=20,center=true);  
}

module deckel (tol)
{
    
    
        difference()
        {
            translate([D/2,D/2,H-D])cube([B-tol-D,L-tol-D,D-tol]);
            
            translate([2.5+1,2.5+1,D])cylinder(d=3,h=H);
            translate([L-(2.5+1),2.5+1,D])cylinder(d=3,h=H);
            translate([2.5+1,B-(2.5+1),D])cylinder(d=3,h=H);
            translate([L-(2.5+1),B-(2.5+1),D])cylinder(d=3,h=H);
            
            translate([46,21,H-D-3])cylinder(d=8,h=10);
            translate([46,21,H-D])cube([15,15,2],center=true);
        }
            
}

module platine ()
{
    cube([2.54*15,2.54*12,1]);
    translate([-9,4*2.54,1])cube([12,2.54*4,3]);
    translate([-9,10*2.54,1])cube([12,2.54*2,3]);
    translate([-9,0*2.54,1])cube([12,2.54*2,3]);
    
    
}

module box ()
{
    difference()
    {
        union()
        {
            difference()
            {
                
                hull(){
                    translate([2,2,0])cylinder(r=2,h=H,$fn=20);
                    translate([B-2,2,0])cylinder(r=2,h=H,$fn=20);
                    translate([2,L-2,0])cylinder(r=2,h=H,$fn=20);
                    translate([L-2,B-2,0])cylinder(r=2,h=H,$fn=20);
                }
               
                translate([D,D,D])cube([B-D*2,L-D*2,H]);  
                translate([25,L,12])rotate([90,0,0])sub_d();
                translate([50,L,9])rotate([90,0,0])cylinder(d=3,h=20,center=true);
                translate([50,L,9+6])rotate([90,0,0])cylinder(d=3,h=20,center=true);
                
            }
            
            difference()
            {
                union()
                {
                    translate([2.5+1,2.5+1,0])cylinder(d=5.5,h=H);
                    translate([L-(2.5+1),2.5+1,0])cylinder(d=5.5,h=H);
                    translate([2.5+1,B-(2.5+1),0])cylinder(d=5.5,h=H);
                    translate([L-(2.5+1),B-(2.5+1),0])cylinder(d=5.5,h=H);
                }
                    translate([2.5+1,2.5+1,D])cylinder(d=3,h=H);
                    translate([L-(2.5+1),2.5+1,D])cylinder(d=3,h=H);
                    translate([2.5+1,B-(2.5+1),D])cylinder(d=3,h=H);
                    translate([L-(2.5+1),B-(2.5+1),D])cylinder(d=3,h=H);
            }
            
            translate([2,11.5,0])cube([10,8,18.5]);
            translate([2,26.5,0])cube([10,8,18.5]);
            translate([35,5,0])cube([10,10,18.5]);
            translate([35,29,0])cube([10,10,18.5]);
        }     
        deckel(0);   
        translate([3,8,23-4])platine();    
    }   
}

box();
//deckel(0.6);
