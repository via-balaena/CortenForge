//! Bill of materials (BOM) generation for assemblies.
//!
//! A BOM provides a structured list of all parts in an assembly
//! with their properties, dimensions, and materials.

use hashbrown::HashMap;
use std::io::Write;
use std::path::Path;

use crate::connection::Connection;
use crate::error::{AssemblyError, AssemblyResult};

/// Bill of materials for an assembly.
///
/// Contains structured information about all parts including
/// dimensions, materials, and relationships.
#[derive(Debug, Clone)]
pub struct BillOfMaterials {
    /// Assembly name.
    pub assembly_name: String,

    /// Assembly version.
    pub version: Option<String>,

    /// List of items in the BOM.
    pub items: Vec<BomItem>,

    /// Connections between parts.
    pub connections: Vec<Connection>,
}

impl BillOfMaterials {
    /// Create a new empty BOM.
    #[must_use]
    pub fn new(assembly_name: impl Into<String>) -> Self {
        Self {
            assembly_name: assembly_name.into(),
            version: None,
            items: Vec::new(),
            connections: Vec::new(),
        }
    }

    /// Get total part count (sum of all quantities).
    #[must_use]
    pub fn total_parts(&self) -> usize {
        self.items.iter().map(|i| i.quantity).sum()
    }

    /// Get unique material names.
    #[must_use]
    pub fn unique_materials(&self) -> Vec<&str> {
        let mut materials: Vec<&str> = self
            .items
            .iter()
            .filter_map(|i| i.material.as_deref())
            .collect();
        materials.sort_unstable();
        materials.dedup();
        materials
    }

    /// Get parts by material.
    #[must_use]
    pub fn parts_by_material(&self, material: &str) -> Vec<&BomItem> {
        self.items
            .iter()
            .filter(|i| i.material.as_deref() == Some(material))
            .collect()
    }

    /// Get a part by ID.
    #[must_use]
    pub fn get_part(&self, part_id: &str) -> Option<&BomItem> {
        self.items.iter().find(|i| i.part_id == part_id)
    }

    /// Export the BOM to a CSV file.
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be written.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use mesh_assembly::BillOfMaterials;
    /// use std::path::Path;
    ///
    /// let bom = BillOfMaterials::new("assembly");
    /// bom.export_csv(Path::new("bom.csv")).unwrap();
    /// ```
    pub fn export_csv(&self, path: &Path) -> AssemblyResult<()> {
        use std::fs::File;

        let file = File::create(path).map_err(|e| AssemblyError::Io {
            path: path.to_path_buf(),
            source: e,
        })?;

        let mut writer = std::io::BufWriter::new(file);

        // Write header
        writeln!(
            writer,
            "Part ID,Material,Quantity,Width (mm),Height (mm),Depth (mm),Volume (mm\u{b3}),Triangles,Parent"
        )
        .map_err(|e| AssemblyError::Io {
            path: path.to_path_buf(),
            source: e,
        })?;

        // Write items
        for item in &self.items {
            writeln!(
                writer,
                "{},{},{},{:.2},{:.2},{:.2},{:.2},{},{}",
                escape_csv(&item.part_id),
                escape_csv(item.material.as_deref().unwrap_or("")),
                item.quantity,
                item.dimensions.0,
                item.dimensions.1,
                item.dimensions.2,
                item.bounding_volume,
                item.triangle_count,
                escape_csv(item.parent.as_deref().unwrap_or(""))
            )
            .map_err(|e| AssemblyError::Io {
                path: path.to_path_buf(),
                source: e,
            })?;
        }

        Ok(())
    }
}

/// A single item in the bill of materials.
#[derive(Debug, Clone)]
pub struct BomItem {
    /// Part ID.
    pub part_id: String,

    /// Display name.
    pub name: String,

    /// Material name.
    pub material: Option<String>,

    /// Quantity.
    pub quantity: usize,

    /// Bounding box dimensions (width, height, depth) in mm.
    pub dimensions: (f64, f64, f64),

    /// Bounding box volume in mm cubed.
    pub bounding_volume: f64,

    /// Number of triangles.
    pub triangle_count: usize,

    /// Parent part ID.
    pub parent: Option<String>,

    /// Additional metadata.
    pub metadata: HashMap<String, String>,
}

impl BomItem {
    /// Create a new BOM item.
    #[must_use]
    pub fn new(part_id: impl Into<String>) -> Self {
        let id = part_id.into();
        Self {
            name: id.clone(),
            part_id: id,
            material: None,
            quantity: 1,
            dimensions: (0.0, 0.0, 0.0),
            bounding_volume: 0.0,
            triangle_count: 0,
            parent: None,
            metadata: HashMap::new(),
        }
    }
}

/// Escape special characters for CSV.
fn escape_csv(s: &str) -> String {
    if s.contains(',') || s.contains('"') || s.contains('\n') {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bom_new() {
        let bom = BillOfMaterials::new("test_assembly");
        assert_eq!(bom.assembly_name, "test_assembly");
        assert!(bom.items.is_empty());
        assert_eq!(bom.total_parts(), 0);
    }

    #[test]
    fn test_bom_total_parts() {
        let mut bom = BillOfMaterials::new("test");
        bom.items.push(BomItem {
            part_id: "a".to_string(),
            name: "a".to_string(),
            material: None,
            quantity: 2,
            dimensions: (0.0, 0.0, 0.0),
            bounding_volume: 0.0,
            triangle_count: 0,
            parent: None,
            metadata: HashMap::new(),
        });
        bom.items.push(BomItem {
            part_id: "b".to_string(),
            name: "b".to_string(),
            material: None,
            quantity: 3,
            dimensions: (0.0, 0.0, 0.0),
            bounding_volume: 0.0,
            triangle_count: 0,
            parent: None,
            metadata: HashMap::new(),
        });

        assert_eq!(bom.total_parts(), 5);
    }

    #[test]
    fn test_unique_materials() {
        let mut bom = BillOfMaterials::new("test");
        bom.items.push(BomItem {
            part_id: "a".to_string(),
            name: "a".to_string(),
            material: Some("PLA".to_string()),
            quantity: 1,
            dimensions: (0.0, 0.0, 0.0),
            bounding_volume: 0.0,
            triangle_count: 0,
            parent: None,
            metadata: HashMap::new(),
        });
        bom.items.push(BomItem {
            part_id: "b".to_string(),
            name: "b".to_string(),
            material: Some("TPU".to_string()),
            quantity: 1,
            dimensions: (0.0, 0.0, 0.0),
            bounding_volume: 0.0,
            triangle_count: 0,
            parent: None,
            metadata: HashMap::new(),
        });
        bom.items.push(BomItem {
            part_id: "c".to_string(),
            name: "c".to_string(),
            material: Some("PLA".to_string()),
            quantity: 1,
            dimensions: (0.0, 0.0, 0.0),
            bounding_volume: 0.0,
            triangle_count: 0,
            parent: None,
            metadata: HashMap::new(),
        });

        let materials = bom.unique_materials();
        assert_eq!(materials.len(), 2);
        assert!(materials.contains(&"PLA"));
        assert!(materials.contains(&"TPU"));
    }

    #[test]
    fn test_parts_by_material() {
        let mut bom = BillOfMaterials::new("test");
        bom.items.push(BomItem {
            part_id: "a".to_string(),
            name: "a".to_string(),
            material: Some("PLA".to_string()),
            quantity: 1,
            dimensions: (0.0, 0.0, 0.0),
            bounding_volume: 0.0,
            triangle_count: 0,
            parent: None,
            metadata: HashMap::new(),
        });
        bom.items.push(BomItem {
            part_id: "b".to_string(),
            name: "b".to_string(),
            material: Some("TPU".to_string()),
            quantity: 1,
            dimensions: (0.0, 0.0, 0.0),
            bounding_volume: 0.0,
            triangle_count: 0,
            parent: None,
            metadata: HashMap::new(),
        });

        let pla_parts = bom.parts_by_material("PLA");
        assert_eq!(pla_parts.len(), 1);
        assert_eq!(pla_parts[0].part_id, "a");
    }

    #[test]
    fn test_get_part() {
        let mut bom = BillOfMaterials::new("test");
        bom.items.push(BomItem::new("part1"));
        bom.items.push(BomItem::new("part2"));

        assert!(bom.get_part("part1").is_some());
        assert!(bom.get_part("part2").is_some());
        assert!(bom.get_part("nonexistent").is_none());
    }

    #[test]
    fn test_escape_csv() {
        assert_eq!(escape_csv("simple"), "simple");
        assert_eq!(escape_csv("with,comma"), "\"with,comma\"");
        assert_eq!(escape_csv("with\"quote"), "\"with\"\"quote\"");
        assert_eq!(escape_csv("with\nnewline"), "\"with\nnewline\"");
    }

    #[test]
    fn test_bom_item_new() {
        let item = BomItem::new("test_part");
        assert_eq!(item.part_id, "test_part");
        assert_eq!(item.name, "test_part");
        assert_eq!(item.quantity, 1);
        assert!(item.material.is_none());
    }
}
