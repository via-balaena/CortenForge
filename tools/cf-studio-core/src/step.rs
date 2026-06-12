//! The six ordered workflow steps.
//!
//! [`Step`] derives `Ord` by declaration order, so `AddScan < Pour`
//! holds and the state machine can reason about "earlier" / "later"
//! directly. All navigation helpers are `const` and total — there are
//! no panicking paths.

use serde::{Deserialize, Serialize};

/// One step of the guided scan→cast workflow. Ordered first → last.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Step {
    /// Load a 3-D scan of the area.
    AddScan,
    /// Repair + seal the scan into a castable, watertight body.
    CleanScan,
    /// Cavity inset + the soft→firm silicone layer stack.
    DesignLayers,
    /// Optional surface texture: interior ridges (the plug) + exterior /
    /// inter-layer shell ridges. Skippable — defaults to no texture.
    Texture,
    /// Generate the printable mold pieces + plugs.
    MakeMolds,
    /// Export the mold files for the 3-D printer.
    Print,
    /// The guided silicone-pour assistant.
    Pour,
}

impl Step {
    /// Every step, in workflow order.
    pub const ALL: [Step; 7] = [
        Step::AddScan,
        Step::CleanScan,
        Step::DesignLayers,
        Step::Texture,
        Step::MakeMolds,
        Step::Print,
        Step::Pour,
    ];

    /// The first step.
    pub const FIRST: Step = Step::AddScan;

    /// The final step.
    pub const LAST: Step = Step::Pour;

    /// Total number of steps (for "Step N of [`TOTAL`](Step::TOTAL)").
    pub const TOTAL: usize = 7;

    /// Zero-based position of this step in the workflow.
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Step::AddScan => 0,
            Step::CleanScan => 1,
            Step::DesignLayers => 2,
            Step::Texture => 3,
            Step::MakeMolds => 4,
            Step::Print => 5,
            Step::Pour => 6,
        }
    }

    /// One-based step number, for display ("Step 3 of 7").
    #[must_use]
    pub const fn number(self) -> usize {
        self.index() + 1
    }

    /// The next step, or `None` if this is the last.
    #[must_use]
    pub const fn next(self) -> Option<Step> {
        match self {
            Step::AddScan => Some(Step::CleanScan),
            Step::CleanScan => Some(Step::DesignLayers),
            Step::DesignLayers => Some(Step::Texture),
            Step::Texture => Some(Step::MakeMolds),
            Step::MakeMolds => Some(Step::Print),
            Step::Print => Some(Step::Pour),
            Step::Pour => None,
        }
    }

    /// The previous step, or `None` if this is the first.
    #[must_use]
    pub const fn prev(self) -> Option<Step> {
        match self {
            Step::AddScan => None,
            Step::CleanScan => Some(Step::AddScan),
            Step::DesignLayers => Some(Step::CleanScan),
            Step::Texture => Some(Step::DesignLayers),
            Step::MakeMolds => Some(Step::Texture),
            Step::Print => Some(Step::MakeMolds),
            Step::Pour => Some(Step::Print),
        }
    }

    /// Short, plain-language title shown to a non-technical user.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Step::AddScan => "Add your scan",
            Step::CleanScan => "Clean up the scan",
            Step::DesignLayers => "Choose how it should feel",
            Step::Texture => "Add surface texture",
            Step::MakeMolds => "Make your molds",
            Step::Print => "3D print the molds",
            Step::Pour => "Pour the silicone",
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    #[test]
    fn declaration_order_is_workflow_order() {
        assert!(Step::AddScan < Step::CleanScan);
        assert!(Step::CleanScan < Step::DesignLayers);
        assert!(Step::DesignLayers < Step::Texture);
        assert!(Step::Texture < Step::MakeMolds);
        assert!(Step::MakeMolds < Step::Print);
        assert!(Step::Print < Step::Pour);
    }

    #[test]
    fn all_is_sorted_and_complete() {
        assert_eq!(Step::ALL.len(), Step::TOTAL);
        let mut sorted = Step::ALL;
        sorted.sort_unstable();
        assert_eq!(sorted, Step::ALL, "ALL must already be in workflow order");
        assert_eq!(Step::ALL[0], Step::FIRST);
        assert_eq!(Step::ALL[Step::TOTAL - 1], Step::LAST);
    }

    #[test]
    fn index_matches_position_in_all() {
        for (i, step) in Step::ALL.iter().enumerate() {
            assert_eq!(step.index(), i);
            assert_eq!(step.number(), i + 1);
        }
    }

    #[test]
    fn next_prev_form_a_consistent_chain() {
        assert_eq!(Step::FIRST.prev(), None);
        assert_eq!(Step::LAST.next(), None);
        let mut step = Step::FIRST;
        while let Some(next) = step.next() {
            assert_eq!(next.prev(), Some(step), "prev must invert next");
            assert!(next > step, "next must move forward");
            step = next;
        }
        assert_eq!(step, Step::LAST, "walking next() must terminate at LAST");
    }

    #[test]
    fn every_step_has_a_nonempty_title() {
        for step in Step::ALL {
            assert!(!step.title().is_empty(), "{step:?} has no title");
        }
    }

    #[test]
    fn round_trips_through_serde() {
        for step in Step::ALL {
            let json = serde_json::to_string(&step).unwrap();
            let back: Step = serde_json::from_str(&json).unwrap();
            assert_eq!(step, back);
        }
        // Fieldless variants serialize as their bare name.
        assert_eq!(
            serde_json::to_string(&Step::MakeMolds).unwrap(),
            "\"MakeMolds\""
        );
    }
}
