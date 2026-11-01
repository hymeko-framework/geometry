//! Source handles carried by a kinematic model.
//!
//! A model read from a description keeps that description's own identifiers on its links,
//! joints, attachments and drives, so a consumer maps a part back to what declared it without a
//! name lookup. Geometry never interprets them: it compares, hashes and copies them, and treats
//! [`Handles::NONE`] as "no source declaration" (hand-built models, unresolved references), in
//! which case names decide. The reader picks the handle type; HyMeKo's reader in the
//! transformation repository uses Core declaration ids.

use std::fmt::Debug;
use std::hash::Hash;

/// The handle family of one model source.
pub trait Handles: Clone + Debug {
    /// One declaration's identifier.
    type Id: Copy + Eq + Hash + Debug;
    /// The identifier of a part that has no source declaration.
    const NONE: Self::Id;
}

/// Plain indices, for models built without a description.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Indexed;

impl Handles for Indexed {
    type Id = usize;
    const NONE: usize = usize::MAX;
}
