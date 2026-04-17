// Parameters
number_text = "IPB";     // Change this to any number or text
font_size = 15;        // Size of the text
thickness = 5;         // Extrusion height

// Parameters
number_text = "0";     // Change this to any number or text
font_size = 20;        // Size of the text
thickness = 5;         // Extrusion height

// 3D text
linear_extrude(height = thickness)
    text(number_text,
         size = font_size,
         font = "Liberation Sans:style=Bold",
         halign = "center",
         valign = "center");