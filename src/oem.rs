/// FLIR Lepton OEM module video output source selector.
///
/// The values map to the OEM CCI command pair at module `0x0800`, command base
/// `0x2C/0x2D` (Get/Set). These modes are useful for link diagnostics because
/// they produce deterministic pixel patterns that can be validated over VoSPI.
///
/// See Lepton Software IDD Rev 303 (OEM Video Output Source Select).
#[repr(u16)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VideoOutputSource {
    Raw = 0,
    Cooked = 1,
    Ramp = 2,
    Constant = 3,
    RampH = 4,
    RampV = 5,
}
