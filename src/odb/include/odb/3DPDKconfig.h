// SPDX-License-Identifier: BSD-3-Clause
// Config_3DPDK.h
#pragma once

#include <vector>
#include <cstdint>
#include <map>
#include <ostream>
#include <sstream>
#include <string>
#include "utl/Logger.h"
#include <unordered_map>

namespace odb {

class Config_3DPDK
{
public:
    // =========================================================
    // Structs
    // =========================================================

    struct TierInfo {
        std::string id;
        std::string name;
        std::string role;
        std::string orientation;
        std::string pdk_name;
        std::string pdk_root;
        std::string db_path;
        std::vector<std::string> liberty_files;
        double thickness = 0.0;
        double offset_z = 0.0;
        bool flip_x = false;
        bool flip_y = false;
        std::string min_metal_layer;
        std::string max_metal_layer;
        int preferred_tracks = 0;
        std::string vdd_net;
        std::string gnd_net;
        double strap_pitch = 0.0;
        std::string pad_ring;
    };

    struct InterconnectInfo {
        std::string type;  // TSV, Microbump, etc.
        std::string name;
        std::string lef_path;
        // --- New fields for enclosure ---
        std::pair<double, double> bottom_enclosure {0.0, 0.0};  // XBottom, YBottom
        std::pair<double, double> top_enclosure {0.0, 0.0};     // XTop, YTop
    };

    struct IntegrationLink {
        std::string from_tier;
        std::string to_tier;
        std::string integration_type;  // F2F, F2B, B2B
        std::string bonding_mode;
        std::string bonding_interface_layer;
        std::vector<InterconnectInfo> interconnects;
    };

private:
    // =========================================================
    // Member Variables
    // =========================================================
    utl::Logger* logger_;
    std::string openroad_root_;
    std::string workspace_dir_;
    std::string integration_mode_;
    std::string tracks_file_path_;
    double tolerance_xy_ = 0.0;
    double tolerance_z_ = 0.0;

    std::vector<TierInfo> tiers_;
    std::vector<IntegrationLink> integration_stack_;

    std::string bonding_mode_;
    std::string bonding_interface_layer_;

    std::string composite_tech_lef_;
    std::string composite_combined_lef_;
    std::string composite_combined_lib_;
    double vertical_spacing_ = 0.0;

    std::string inter_tier_rc_model_;
    double coupling_coefficient_ = 0.0;

    std::string thermal_interface_material_;
    double thermal_interface_thickness_ = 0.0;
    std::map<std::string, double> thermal_conductivity_;

public:
    // =========================================================
    // Constructors
    // =========================================================
    Config_3DPDK(utl::Logger* logger);

    static Config_3DPDK parse3DPDKconfigfile(utl::Logger* logger, const std::string& filename);

    // =========================================================
    // Getters
    // =========================================================
    std::string getOpenroadRoot() const;
    std::string getWorkspaceDir() const;
    std::string getIntegrationMode() const;
    std::string getTracksfilePath() const;
    double getToleranceXY() const;
    double getToleranceZ() const;

    std::vector<TierInfo> getTiers() const;
    std::vector<IntegrationLink> getIntegrationStack() const;

    std::string getBondingMode() const;
    std::string getBondingInterfaceLayer() const;

    std::string getCompositeTechLEF() const;
    std::string getCompositeCombinedLEF() const;
    std::string getCompositeCombinedLIB() const;
    double getVerticalSpacing() const;

    std::string getInterTierRCModel() const;
    double getCouplingCoefficient() const;

    std::string getThermalInterfaceMaterial() const;
    double getThermalInterfaceThickness() const;
    std::map<std::string, double> getThermalConductivity() const;

    // =========================================================
    // Setters
    // =========================================================
    void setOpenroadRoot(const std::string& val);
    void setWorkspaceDir(const std::string& val);
    void setIntegrationMode(const std::string& val);
    void setTracksfilePath(const std::string& val);
    void setToleranceXY(double val);
    void setToleranceZ(double val);

    void setTiers(const std::vector<TierInfo>& val);
    void setIntegrationStack(const std::vector<IntegrationLink>& val);

    void setBondingMode(const std::string& val);
    void setBondingInterfaceLayer(const std::string& val);

    void setCompositeTechLEF(const std::string& val);
    void setCompositeCombinedLEF(const std::string& val);
    void setCompositeCombinedLIB(const std::string& val);
    void setVerticalSpacing(double val);

    void setInterTierRCModel(const std::string& val);
    void setCouplingCoefficient(double val);

    void setThermalInterfaceMaterial(const std::string& val);
    void setThermalInterfaceThickness(double val);
    void setThermalConductivity(const std::map<std::string, double>& val);

    // =========================================================
    // Utility
    // =========================================================
    void printSummary() const;

};

}  // namespace odb
