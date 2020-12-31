//
// A maze
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Dev 24 2020
//

// Units are in meters
border_height = 1;
border_size = 5;
border_thickness = 0.25;

interior_size = border_size - (2 * border_thickness);

global_offset = [-1, -1, 0];

bind = 0.01;

module wall(length, width, height) {
    cube(size=[length, width, height]);
}

module interior_walls() {
    translate([0, border_thickness + 1.5, 0])
        wall(3, border_thickness, border_height);
    translate([border_size - 3, border_thickness + 3, 0])
        wall(3, border_thickness, border_height);
}

module border() {
    difference() {
        cube(size=[border_size, border_size, border_height]);
        translate([border_thickness, border_thickness, -bind])
            cube(size=[interior_size, interior_size, border_height + 2 * bind]);
    }
}

module maze() {
    union(){
        border();
        interior_walls();
    }
}

translate(global_offset)
    maze();
