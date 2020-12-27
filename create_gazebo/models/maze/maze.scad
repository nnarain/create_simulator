//
// A maze
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Dev 24 2020
//

// Units are in meters
border_height = 1;
border_size = 5;
border_thickness = 0.5;

interior_size = border_size - (2 * border_thickness);

bind = 0.01;

module maze() {
    difference(){
        cube(size=[border_size, border_size, border_height], center=true);
        translate([0, 0, -bind])
            cube(size=[interior_size, interior_size, border_height + (2.5 * bind)], center=true);
    }
}

maze();
