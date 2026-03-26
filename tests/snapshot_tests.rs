/// Snapshot tests — feed real pcap-extracted fixture files through the full
/// parser pipeline and compare against golden `.expected` files.
///
/// To regenerate expected files after intentional output changes:
///   GSOF_BLESS=1 cargo test --test snapshot_tests
use gsof_parser::reassembly::Reassembler;
use gsof_parser::trimcomm::{FrameParser, TYPE_GSOF};
use std::path::{Path, PathBuf};

fn fixtures_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
}

/// Run the full pipeline on raw Trimcomm bytes: frame -> reassemble -> parse -> Display.
fn parse_fixture(raw: &[u8]) -> String {
    let mut output = String::new();
    let mut parser = FrameParser::new();
    let mut reassembler = Reassembler::new();

    for &byte in raw {
        if let Some(Ok(pkt)) = parser.push(byte) {
            if pkt.packet_type != TYPE_GSOF {
                continue;
            }
            match reassembler.push(&pkt.data) {
                Ok(push) => {
                    if let Some(result) = push.records {
                        match result {
                            Ok(records) => {
                                for rec in &records {
                                    output.push_str(&format!("{rec}\n"));
                                }
                            }
                            Err(e) => output.push_str(&format!("PARSE ERROR: {e}\n")),
                        }
                    }
                }
                Err(e) => output.push_str(&format!("REASSEMBLY ERROR: {e}\n")),
            }
        }
    }

    output
}

fn run_snapshot(fixture_name: &str) {
    let bin_path = fixtures_dir().join(format!("{fixture_name}.bin"));
    let expected_path = fixtures_dir().join(format!("{fixture_name}.expected"));

    let raw = std::fs::read(&bin_path)
        .unwrap_or_else(|e| panic!("Cannot read {}: {e}", bin_path.display()));

    let actual = parse_fixture(&raw);

    if std::env::var("GSOF_BLESS").is_ok() {
        // Bless mode: write actual output as the new expected file
        std::fs::write(&expected_path, &actual)
            .unwrap_or_else(|e| panic!("Cannot write {}: {e}", expected_path.display()));
        eprintln!("BLESSED {}", expected_path.display());
        return;
    }

    let expected = std::fs::read_to_string(&expected_path).unwrap_or_else(|e| {
        panic!(
            "Cannot read {}; run with GSOF_BLESS=1 to generate: {e}",
            expected_path.display()
        )
    });

    if actual != expected {
        // Show a useful diff
        let mut diff = String::new();
        diff.push_str(&format!("SNAPSHOT MISMATCH: {fixture_name}\n"));
        diff.push_str(&format!("--- expected: {}\n", expected_path.display()));
        diff.push_str("+++ actual\n");
        for diffline in diff::lines(&expected, &actual) {
            match diffline {
                diff::Result::Left(l) => diff.push_str(&format!("-{l}\n")),
                diff::Result::Right(r) => diff.push_str(&format!("+{r}\n")),
                diff::Result::Both(b, _) => diff.push_str(&format!(" {b}\n")),
            }
        }
        panic!("{diff}");
    }
}

// One test per fixture — each named after the pcap source.

#[test] fn snapshot_applus60_gsof1()  { run_snapshot("applus60_gsof1"); }
#[test] fn snapshot_applus60_gsof2()  { run_snapshot("applus60_gsof2"); }
#[test] fn snapshot_applus60_gsof3()  { run_snapshot("applus60_gsof3"); }
#[test] fn snapshot_applus60_gsof6()  { run_snapshot("applus60_gsof6"); }
#[test] fn snapshot_applus60_gsof7()  { run_snapshot("applus60_gsof7"); }
#[test] fn snapshot_applus60_gsof8()  { run_snapshot("applus60_gsof8"); }
#[test] fn snapshot_applus60_gsof9()  { run_snapshot("applus60_gsof9"); }
#[test] fn snapshot_applus60_gsof10() { run_snapshot("applus60_gsof10"); }
#[test] fn snapshot_applus60_gsof11() { run_snapshot("applus60_gsof11"); }
#[test] fn snapshot_applus60_gsof12() { run_snapshot("applus60_gsof12"); }
#[test] fn snapshot_applus60_gsof15() { run_snapshot("applus60_gsof15"); }
#[test] fn snapshot_applus60_gsof16() { run_snapshot("applus60_gsof16"); }
#[test] fn snapshot_applus60_gsof27() { run_snapshot("applus60_gsof27"); }
#[test] fn snapshot_applus60_gsof33() { run_snapshot("applus60_gsof33"); }
#[test] fn snapshot_applus60_gsof34() { run_snapshot("applus60_gsof34"); }
#[test] fn snapshot_applus60_gsof35() { run_snapshot("applus60_gsof35"); }
#[test] fn snapshot_applus60_gsof37() { run_snapshot("applus60_gsof37"); }
#[test] fn snapshot_applus60_gsof38() { run_snapshot("applus60_gsof38"); }
#[test] fn snapshot_applus60_gsof40() { run_snapshot("applus60_gsof40"); }
#[test] fn snapshot_applus60_gsof41() { run_snapshot("applus60_gsof41"); }

#[test] fn snapshot_apx18_fullnav()       { run_snapshot("apx_18_fullnavinfo_single"); }
#[test] fn snapshot_apx18_fullrms()       { run_snapshot("apx_18_fullrmsinfo_single"); }
#[test] fn snapshot_apx18_fullnav_rms()   { run_snapshot("apx_18_fullnav_fullrmsinfo"); }
#[test] fn snapshot_apx18_dmi()           { run_snapshot("apx_18_gsof52_dmirawdata_single"); }

#[test] fn snapshot_lvx_fullnav_rms()     { run_snapshot("lvx_fullnav_fullrms"); }
#[test] fn snapshot_lvx_fullnav_rms_2p()  { run_snapshot("lvx_fullnav_fullrms_2packets"); }
#[test] fn snapshot_lvx_multi_message()   { run_snapshot("lvx_multi_message"); }

#[test] fn snapshot_applus20_small()      { run_snapshot("applus20_small_tx"); }
#[test] fn snapshot_applus20_large()      { run_snapshot("applus20_large_tx"); }
#[test] fn snapshot_sps986_gsof1()        { run_snapshot("SPS986_gsof1"); }
