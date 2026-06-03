extern crate relaxed_ik_lib;

use relaxed_ik_lib::relaxed_ik;
use relaxed_ik_lib::utils_rust::file_utils::*;
use nalgebra::{Vector3, UnitQuaternion, Quaternion};

use std::io::{self, BufRead, Write};

fn main() {
    // ---- init RelaxedIK once ----
    let path_to_src = get_path_to_src();
    let default_path_to_setting = path_to_src + "configs/settings.yaml";
    let mut rik = relaxed_ik::RelaxedIK::load_settings(default_path_to_setting.as_str());

    eprintln!("RelaxedIK server ready.");
    eprintln!("Input per line: x y z qx qy qz qw  (metres + quaternion)");

    let stdin = io::stdin();
    let mut stdout = io::stdout();

    for line in stdin.lock().lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => continue,
        };
        let line = line.trim();
        if line.is_empty() {
            continue;
        }

        // Parse 7 floats: x y z qx qy qz qw
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() != 7 {
            eprintln!("Expected 7 numbers, got {}: '{}'", parts.len(), line);
            continue;
        }

        let parse_f = |s: &str| s.parse::<f64>();
        let x = match parse_f(parts[0]) { Ok(v) => v, Err(_) => { eprintln!("Bad x"); continue; } };
        let y = match parse_f(parts[1]) { Ok(v) => v, Err(_) => { eprintln!("Bad y"); continue; } };
        let z = match parse_f(parts[2]) { Ok(v) => v, Err(_) => { eprintln!("Bad z"); continue; } };
        let qx = match parse_f(parts[3]) { Ok(v) => v, Err(_) => { eprintln!("Bad qx"); continue; } };
        let qy = match parse_f(parts[4]) { Ok(v) => v, Err(_) => { eprintln!("Bad qy"); continue; } };
        let qz = match parse_f(parts[5]) { Ok(v) => v, Err(_) => { eprintln!("Bad qz"); continue; } };
        let qw = match parse_f(parts[6]) { Ok(v) => v, Err(_) => { eprintln!("Bad qw"); continue; } };

        // Set the goal for ALL chains (safe default)
        for j in 0..rik.vars.robot.num_chains {
            rik.vars.goal_positions[j] = Vector3::new(x, y, z);

            // Many RelaxedIK builds also store goal orientations.
            // If your build uses a different field name, cargo will tell us and we’ll adjust.
            let q = Quaternion::new(qw, qx, qy, qz);
            rik.vars.goal_quats[j] = UnitQuaternion::from_quaternion(q);
        }

        let sol = rik.solve();

        // Print space-separated joint angles (one line)
        // Example: "0.1 -0.2 1.0 ..."
        for (i, v) in sol.iter().enumerate() {
            if i > 0 {
                write!(stdout, " ").ok();
            }
            write!(stdout, "{:.10}", v).ok();
        }
        writeln!(stdout).ok();
        stdout.flush().ok();
    }
}
