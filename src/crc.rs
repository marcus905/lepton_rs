/// CRC-16/CCITT polynomial used by Lepton VoSPI packet validation.
const CRC16_POLY: u16 = 0x1021;

/// Computes a CRC-16 over a packet using the FLIR VoSPI normalization rules:
/// - bytes [2] and [3] (CRC field) are treated as zero
/// - upper nibble of byte [0] is treated as zero
///
/// Returns `None` when the packet is too short to contain a VoSPI header.
pub fn lepton_packet_crc16_spec(packet: &[u8]) -> Option<u16> {
    if packet.len() < 4 {
        return None;
    }

    let mut crc = 0u16;

    for (idx, &byte) in packet.iter().enumerate() {
        let normalized = match idx {
            0 => byte & 0x0F,
            2 | 3 => 0,
            _ => byte,
        };

        crc ^= (normalized as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
    }

    Some(crc)
}

#[deprecated(note = "Use lepton_packet_crc16_spec, which follows FLIR VoSPI normalization rules")]
pub fn lepton_packet_crc16(packet: &[u8]) -> u16 {
    lepton_packet_crc16_spec(packet).unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::lepton_packet_crc16_spec;

    #[test]
    fn crc_short_packet_returns_none() {
        assert_eq!(lepton_packet_crc16_spec(&[1, 2, 3]), None);
    }

    #[test]
    fn crc_masks_id_upper_nibble() {
        let mut packet_a = [0u8; 12];
        let mut packet_b = packet_a;

        packet_a[0] = 0x10;
        packet_a[1] = 0x14;
        packet_a[4] = 0x5A;

        packet_b[0] = 0xA0;
        packet_b[1] = 0x14;
        packet_b[4] = 0x5A;

        assert_eq!(
            lepton_packet_crc16_spec(&packet_a),
            lepton_packet_crc16_spec(&packet_b)
        );
    }

    #[test]
    fn crc_zeros_crc_field_bytes() {
        let mut packet_a = [0u8; 12];
        packet_a[0] = 0x00;
        packet_a[1] = 0x14;
        packet_a[4] = 0xBE;

        let mut packet_b = packet_a;
        packet_b[2] = 0x12;
        packet_b[3] = 0x34;

        assert_eq!(
            lepton_packet_crc16_spec(&packet_a),
            lepton_packet_crc16_spec(&packet_b)
        );
    }

    #[test]
    fn crc_known_vector() {
        let packet = [0x10, 0x14, 0x00, 0x00, 0xAB, 0xCD, 0x10, 0x20, 0x30, 0x40];
        assert_eq!(lepton_packet_crc16_spec(&packet), Some(0x2F69));
    }
}
