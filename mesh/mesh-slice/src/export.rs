//! Export functionality for sliced layers.

use std::fmt::Write;

use crate::layer::Layer;

/// Parameters for SVG export.
#[derive(Debug, Clone)]
pub struct SvgExportParams {
    /// Width of the SVG in pixels.
    pub width: u32,
    /// Height of the SVG in pixels.
    pub height: u32,
    /// Padding around the content in pixels.
    pub padding: u32,
    /// Stroke width for contours.
    pub stroke_width: f64,
    /// Fill color for solid regions (CSS color string).
    pub fill_color: String,
    /// Stroke color for contours.
    pub stroke_color: String,
    /// Background color.
    pub background_color: String,
    /// Whether to show outer contours filled.
    pub fill_outer: bool,
    /// Whether to show hole contours.
    pub show_holes: bool,
}

impl Default for SvgExportParams {
    fn default() -> Self {
        Self {
            width: 800,
            height: 600,
            padding: 20,
            stroke_width: 1.0,
            fill_color: "#4a90d9".to_string(),
            stroke_color: "#2d5986".to_string(),
            background_color: "#f5f5f5".to_string(),
            fill_outer: true,
            show_holes: true,
        }
    }
}

impl SvgExportParams {
    /// Create params with custom colors.
    #[must_use]
    pub fn with_colors(mut self, fill: &str, stroke: &str) -> Self {
        self.fill_color = fill.to_string();
        self.stroke_color = stroke.to_string();
        self
    }

    /// Create params with custom size.
    #[must_use]
    pub const fn with_size(mut self, width: u32, height: u32) -> Self {
        self.width = width;
        self.height = height;
        self
    }
}

/// Export a single layer to SVG format.
///
/// # Arguments
///
/// * `layer` - The layer to export
/// * `params` - Export parameters
///
/// # Returns
///
/// A string containing the SVG content.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_slice::{slice_preview, SliceParams, export_layer_svg, SvgExportParams};
///
/// let cube = unit_cube();
/// let layer = slice_preview(&cube, 0.5, &SliceParams::default());
/// let svg = export_layer_svg(&layer, &SvgExportParams::default());
/// assert!(svg.contains("<svg"));
/// ```
#[must_use]
pub fn export_layer_svg(layer: &Layer, params: &SvgExportParams) -> String {
    if layer.contours.is_empty() {
        return format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{}\" height=\"{}\" viewBox=\"0 0 {} {}\">\n\
  <rect width=\"100%\" height=\"100%\" fill=\"{}\"/>\n\
  <text x=\"50%\" y=\"50%\" text-anchor=\"middle\" fill=\"#999\">Empty layer</text>\n\
</svg>",
            params.width, params.height, params.width, params.height, params.background_color
        );
    }

    // Calculate bounds and scale
    let bounds = &layer.bounds;
    let content_width = bounds.width();
    let content_height = bounds.height();

    let padding = f64::from(params.padding);
    let available_width = 2.0f64.mul_add(-padding, f64::from(params.width));
    let available_height = 2.0f64.mul_add(-padding, f64::from(params.height));

    let scale = if content_width > 0.0 && content_height > 0.0 {
        (available_width / content_width).min(available_height / content_height)
    } else {
        1.0
    };

    let offset_x = padding + content_width.mul_add(-scale, available_width) / 2.0;
    let offset_y = padding + content_height.mul_add(-scale, available_height) / 2.0;

    let mut svg = format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">
  <rect width="100%" height="100%" fill="{}"/>
  <g transform="translate({:.2},{:.2}) scale({:.6},{:.6})">
"#,
        params.width,
        params.height,
        params.width,
        params.height,
        params.background_color,
        bounds.min_x.mul_add(-scale, offset_x),
        bounds.max_y.mul_add(scale, offset_y), // SVG Y is inverted
        scale,
        -scale // Flip Y axis
    );

    // Draw contours
    for contour in &layer.contours {
        if contour.points.is_empty() {
            continue;
        }

        if !contour.is_outer && !params.show_holes {
            continue;
        }

        let mut path = String::new();
        for (i, point) in contour.points.iter().enumerate() {
            if i == 0 {
                let _ = write!(path, "M {:.4} {:.4}", point.x, point.y);
            } else {
                let _ = write!(path, " L {:.4} {:.4}", point.x, point.y);
            }
        }
        path.push_str(" Z");

        let fill = if params.fill_outer && contour.is_outer {
            &params.fill_color
        } else if !contour.is_outer {
            &params.background_color // Holes cut out
        } else {
            "none"
        };

        let _ = writeln!(
            svg,
            r#"    <path d="{}" fill="{}" stroke="{}" stroke-width="{:.2}"/>"#,
            path,
            fill,
            params.stroke_color,
            params.stroke_width / scale
        );
    }

    svg.push_str("  </g>\n");

    // Add layer info text
    let _ = write!(
        svg,
        "  <text x=\"10\" y=\"20\" font-family=\"monospace\" font-size=\"12\" fill=\"#666\">\n\
    Layer {}: Z={:.2}mm, Area={:.1}mm², Perimeter={:.1}mm\n\
  </text>\n",
        layer.index, layer.z_height, layer.area, layer.perimeter
    );

    svg.push_str("</svg>");

    svg
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::layer::{Contour, LayerBounds};
    use mesh_types::Point3;

    fn create_test_layer() -> Layer {
        Layer {
            index: 0,
            z_height: 1.0,
            thickness: 0.2,
            contours: vec![Contour {
                points: vec![
                    Point3::new(0.0, 0.0, 1.0),
                    Point3::new(10.0, 0.0, 1.0),
                    Point3::new(10.0, 10.0, 1.0),
                    Point3::new(0.0, 10.0, 1.0),
                ],
                area: 100.0,
                perimeter: 40.0,
                is_outer: true,
                centroid: Point3::new(5.0, 5.0, 1.0),
            }],
            area: 100.0,
            perimeter: 40.0,
            print_time: 60.0,
            filament_length: 100.0,
            island_count: 1,
            bounds: LayerBounds {
                min_x: 0.0,
                max_x: 10.0,
                min_y: 0.0,
                max_y: 10.0,
            },
        }
    }

    #[test]
    fn test_svg_export_params_default() {
        let params = SvgExportParams::default();
        assert_eq!(params.width, 800);
        assert_eq!(params.height, 600);
        assert_eq!(params.padding, 20);
        assert!(params.fill_outer);
        assert!(params.show_holes);
    }

    #[test]
    fn test_svg_export_params_builder() {
        let params = SvgExportParams::default()
            .with_colors("#ff0000", "#000000")
            .with_size(1024, 768);

        assert_eq!(params.fill_color, "#ff0000");
        assert_eq!(params.stroke_color, "#000000");
        assert_eq!(params.width, 1024);
        assert_eq!(params.height, 768);
    }

    #[test]
    fn test_export_layer_svg_empty() {
        let layer = Layer {
            index: 0,
            z_height: 0.0,
            thickness: 0.2,
            contours: vec![],
            area: 0.0,
            perimeter: 0.0,
            print_time: 0.0,
            filament_length: 0.0,
            island_count: 0,
            bounds: LayerBounds::default(),
        };
        let params = SvgExportParams::default();
        let svg = export_layer_svg(&layer, &params);

        assert!(svg.contains("<svg"));
        assert!(svg.contains("Empty layer"));
        assert!(svg.contains("</svg>"));
    }

    #[test]
    fn test_export_layer_svg_with_contour() {
        let layer = create_test_layer();
        let params = SvgExportParams::default();
        let svg = export_layer_svg(&layer, &params);

        assert!(svg.contains("<svg"));
        assert!(svg.contains("</svg>"));
        assert!(svg.contains("<path"));
        assert!(svg.contains("Layer 0:"));
    }
}
