# 🧱 OpenROAD 3D PDK Creation Framework

## Overview

The **OpenROAD 3D PDK Creation Framework** extends the OpenROAD ecosystem to enable **native 3D Process Design Kit (3D PDK)** creation and integration.  
It provides an automated flow for combining multiple 2D PDKs (LEF, Liberty, and DB files) into a cohesive **multi-tier 3D technology stack**.

This framework introduces a **YAML-based configuration system (`*.config`)** that describes the entire 3D integration hierarchy — from per-tier definitions to bonding and interconnect details.

With this framework, designers can:
- Define **tiers** (logic, memory, analog, etc.) with independent PDKs.
- Specify **integration modes** (F2F, F2B, B2B, or hybrid).
- Describe **inter-tier interconnects** such as TSVs or microbumps.
- Include **bonding and enclosure information** for accurate physical modeling.
- Automatically generate **3D-aware LEF, Liberty, and routing files** from the configuration.

In essence, this framework transforms OpenROAD into a **3D PDK synthesis and integration platform**, bridging 2D PDK assets into unified 3D-compatible design environments.


## ✳️ Key Features

### 🧩 3D Configuration Manager (`Config_3DPDK`)
- Parses **YAML-based configuration files** describing tiers, interconnects, and bonding.
- Stores **technology, geometry, and hierarchy information** for each tier.
- Supports direct loading of **`.db`**, **LEF**, and **Liberty** files per tier.
- Provides structured access to inter-tier connections, enclosures, and bonding metadata.
- Enables consistent configuration transfer across OpenROAD modules.

### 🏗️ 3D PDK Writer (`write3DPDK`)
- Loads the `.db` files of all tiers into **independent OpenDB databases**.
- Extracts **`dbTech`** and **`dbLib`** objects for each tier.
- Generates a **unified 3D PDK representation**, including:
  - Combined 3D LEFs
  - Merged Liberty files
  - Tier-specific and global routing tracks (`tracks.tcl`)

### 🧾 LEF/Lib Writer (`lefout`)
- Writes **per-tier and merged 3D LEF** files.
- Integrates **TSV** and **microbump** LEFs into the composite 3D stack.
- Automatically generates **routing track definitions** for each metal layer.
- Logs all generation steps through `logger_` for traceability and debugging.

### 🧠 Smart Logging and Data Management
- Centralized logging system using `logger_` for info, warning, and error tracking.
- Tier databases are isolated to **prevent symbol and naming conflicts**.
- Full compatibility with existing OpenROAD TCL workflows and database structures.

### ⚡ Automation and Extensibility
- Fully scriptable through the **`write_3DPDK` TCL command**.
- Supports incremental tier addition or removal via YAML edits.
- Easily integrates into existing OpenROAD design automation flows.


## ⚙️ Configuration File Structure (`.config`)

Each 3D PDK project is described using a single YAML configuration file (`*.config`).  
This file defines all tiers, integration modes, interconnects, and their technology dependencies.

---

### 🧾 Example Configuration

```yaml
openroad_root: /path/to/OpenROAD
workspace_dir: /path/to/3D_workspace
integration_mode: hybrid
tracks_file_path: /path/to/3D_workspace/tracks.tcl

tiers:
  - id: T0
    name: logic_tier
    role: logic
    orientation: R0
    pdk:
      name: nangate45
      root: /path/to/nangate45
      db_path: /path/to/2D_PDK_Nangate45.db
      liberty_files:
        - path: /path/to/Nangate45_typ.lib
        - path: /path/to/Nangate45_fast.lib
        - path: /path/to/Nangate45_slow.lib
    thickness: 20.0
    offset_z: 0.0
    flip_x: false
    flip_y: false
    min_metal_layer: metal1
    max_metal_layer: metal10
    vdd_net: VDD
    gnd_net: VSS

integration:
  stack:
    - from_tier: logic_tier
      to_tier: memory_tier
      integration_type: F2F
      bonding_mode: hybrid_bond
      bonding_interface_layer: BOND_LYR
      interconnects:
        - type: TSV
          name: TSV_L2
          lef_path: /path/to/TSV.lef
          bottom_enclosure: [2.0, 2.0]
          top_enclosure: [3.0, 3.0]

| Section               | Description                                                                                        |
| --------------------- | -------------------------------------------------------------------------------------------------- |
| **openroad_root**     | Path to the OpenROAD installation.                                                                 |
| **workspace_dir**     | Base directory for generated outputs (LEF, LIB, tracks, etc.).                                     |
| **integration_mode**  | Defines the 3D integration type (e.g., `F2F`, `F2B`, `hybrid`).                                    |
| **tiers**             | List of technology tiers, each representing one physical layer stack (logic, memory, cache, etc.). |
| **pdk**               | Contains the tier’s database and library file paths.                                               |
| **integration.stack** | Describes the vertical interconnections and bonding between tiers.                                 |
| **interconnects**     | Defines TSVs, microbumps, or hybrid interfaces, including enclosure geometry and LEF paths.        |

⚠️ YAML Formatting Notes

Indentation is critical — use two spaces per level.
Keys like pdk, tiers, and integration must be followed by a colon (:).
Paths should use forward slashes /, and if spaces are present, wrap them in quotes.
File extensions must match the expected format (.lef, .lib, .db, .config).

## 🧩 Usage

Once your `.config` file is ready, you can generate a full 3D PDK using OpenROAD’s built-in TCL command.

---

### 🧠 TCL Command (inside OpenROAD)

```tcl
write_3DPDK path/to/your_config_file.config

⚙️ Execution Flow

Parses the YAML configuration
The framework reads the .config file and validates the tier, interconnect, and bonding definitions.

Loads tier databases
Each tier’s .db file is independently loaded into its own odb::dbDatabase instance to prevent symbol conflicts.

Extracts and merges LEFs and LIBs
Each tier’s dbTech and dbLib data are collected and combined into cohesive 3D technology files.

Generates routing track definitions
The framework extracts metal layer pitch and offset information and writes it to a tracks.tcl file.

Logs progress
All information, warnings, and errors are reported using the logger_ interface in OpenROAD’s console.

📦 Output Files

Generated inside your workspace directory (e.g., /3D_workspace/outputs/):

composite_3d.lef          # Unified 3D technology LEF
combined_tiers.lef        # Merged LEF from all tiers
combined_tiers.lib        # Aggregated Liberty library
tracks.tcl                # Routing track configuration
tier_reports/             # Logs per tier
 ├── logic_tier.log
 ├── cache_tier.log
 └── memory_tier.log

🧩 Example Run
openroad
openroad> write_3DPDK test/3DPDK_Nangate45.config
[INFO ODB-2000] ==============================================
[INFO ODB-2001] Starting 3D PDK Write Process
[INFO ODB-2002] ==============================================
[INFO ODB-2003] STEP 1: Parsing 3D PDK Configuration File: test/3DPDK_Nangate45.config
[INFO ODB-3000] Initialized 3D PDK configuration manager.
[INFO ODB-3005] Reading tier databases and generating 3D LEFs...
[INFO ODB-3010] Output written to: /3D_workspace/outputs/
openroad> exit

