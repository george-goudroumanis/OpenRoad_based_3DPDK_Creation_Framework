// SPDX-License-Identifier: BSD-3-Clause
// Config_3DPDK.cpp

#include "odb/3DPDKconfig.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include "utl/Logger.h"
#include "utl/MakeLogger.h"

namespace odb {

Config_3DPDK::Config_3DPDK(utl::Logger* logger)
    : logger_(logger)
{
  if (!logger_) {
    throw std::invalid_argument("Config_3DPDK requires a valid logger pointer.");
  }

  logger_->info(utl::ODB, 3000, "Initialized 3D PDK configuration manager.");
}
// ---------------------------------------------------------
// Getters
// ---------------------------------------------------------
std::string Config_3DPDK::getOpenroadRoot() const { return openroad_root_; }
std::string Config_3DPDK::getWorkspaceDir() const { return workspace_dir_; }
std::string Config_3DPDK::getIntegrationMode() const { return integration_mode_; }
std::string Config_3DPDK::getTracksfilePath() const { return integration_mode_; }
double Config_3DPDK::getToleranceXY() const { return tolerance_xy_; }
double Config_3DPDK::getToleranceZ() const { return tolerance_z_; }

std::vector<Config_3DPDK::TierInfo> Config_3DPDK::getTiers() const { return tiers_; }
std::vector<Config_3DPDK::IntegrationLink> Config_3DPDK::getIntegrationStack() const { return integration_stack_; }

std::string Config_3DPDK::getBondingMode() const { return bonding_mode_; }
std::string Config_3DPDK::getBondingInterfaceLayer() const { return bonding_interface_layer_; }

std::string Config_3DPDK::getCompositeTechLEF() const { return composite_tech_lef_; }
std::string Config_3DPDK::getCompositeCombinedLEF() const { return composite_combined_lef_; }
std::string Config_3DPDK::getCompositeCombinedLIB() const { return composite_combined_lib_; }
double Config_3DPDK::getVerticalSpacing() const { return vertical_spacing_; }

std::string Config_3DPDK::getInterTierRCModel() const { return inter_tier_rc_model_; }
double Config_3DPDK::getCouplingCoefficient() const { return coupling_coefficient_; }

std::string Config_3DPDK::getThermalInterfaceMaterial() const { return thermal_interface_material_; }
double Config_3DPDK::getThermalInterfaceThickness() const { return thermal_interface_thickness_; }
std::map<std::string, double> Config_3DPDK::getThermalConductivity() const { return thermal_conductivity_; }

// ---------------------------------------------------------
// Setters
// ---------------------------------------------------------
void Config_3DPDK::setOpenroadRoot(const std::string& val) { openroad_root_ = val; }
void Config_3DPDK::setWorkspaceDir(const std::string& val) { workspace_dir_ = val; }
void Config_3DPDK::setIntegrationMode(const std::string& val) { integration_mode_ = val; }
void Config_3DPDK::setTracksfilePath(const std::string& val) { integration_mode_ = val; }
void Config_3DPDK::setToleranceXY(double val) { tolerance_xy_ = val; }
void Config_3DPDK::setToleranceZ(double val) { tolerance_z_ = val; }

void Config_3DPDK::setTiers(const std::vector<TierInfo>& val) { tiers_ = val; }
void Config_3DPDK::setIntegrationStack(const std::vector<IntegrationLink>& val) { integration_stack_ = val; }

void Config_3DPDK::setBondingMode(const std::string& val) { bonding_mode_ = val; }
void Config_3DPDK::setBondingInterfaceLayer(const std::string& val) { bonding_interface_layer_ = val; }

void Config_3DPDK::setCompositeTechLEF(const std::string& val) { composite_tech_lef_ = val; }
void Config_3DPDK::setCompositeCombinedLEF(const std::string& val) { composite_combined_lef_ = val; }
void Config_3DPDK::setCompositeCombinedLIB(const std::string& val) { composite_combined_lib_ = val; }
void Config_3DPDK::setVerticalSpacing(double val) { vertical_spacing_ = val; }

void Config_3DPDK::setInterTierRCModel(const std::string& val) { inter_tier_rc_model_ = val; }
void Config_3DPDK::setCouplingCoefficient(double val) { coupling_coefficient_ = val; }

void Config_3DPDK::setThermalInterfaceMaterial(const std::string& val) { thermal_interface_material_ = val; }
void Config_3DPDK::setThermalInterfaceThickness(double val) { thermal_interface_thickness_ = val; }
void Config_3DPDK::setThermalConductivity(const std::map<std::string, double>& val) { thermal_conductivity_ = val; }

// ---------------------------------------------------------
// Parser 
// ---------------------------------------------------------
Config_3DPDK Config_3DPDK::parse3DPDKconfigfile(utl::Logger* logger, const std::string& filename)
{
    Config_3DPDK cfg(logger);
    YAML::Node root = YAML::LoadFile(filename);

    if (root["workspace_dir"]) cfg.setWorkspaceDir(root["workspace_dir"].as<std::string>());
    if (root["integration_mode"]) cfg.setIntegrationMode(root["integration_mode"].as<std::string>());
    if (root["tracks_file_path"]) cfg.setTracksfilePath(root["tracks_file_path"].as<std::string>());

    if (root["tolerance"]) {
        if (root["tolerance"]["xy"]) cfg.setToleranceXY(root["tolerance"]["xy"].as<double>());
        if (root["tolerance"]["z"]) cfg.setToleranceZ(root["tolerance"]["z"].as<double>());
    }

    // Tiers
    if (root["tiers"]) {
        std::vector<TierInfo> tiers;
        for (const auto& t : root["tiers"]) {
            TierInfo tier;
            tier.id = t["id"].as<std::string>();
            tier.name = t["name"].as<std::string>();
            tier.role = t["role"].as<std::string>();
            tier.orientation = t["orientation"].as<std::string>();

            // Directly map flat keys instead of nested pdk
            tier.pdk_name = t["pdk"]["name"].as<std::string>();
            tier.pdk_root = t["pdk"]["root"].as<std::string>();
            tier.db_path  = t["pdk"]["db_path"].as<std::string>();
            for (const auto& lf : t["pdk"]["liberty_files"]) {
                // Currently just log liberty file paths; actual loading is elsewhere
                std::string lf_path = lf["path"].as<std::string>();
                tier.liberty_files.push_back(lf_path);
            }

            tier.thickness = t["thickness"].as<double>();
            tier.offset_z = t["offset_z"].as<double>();
            tier.flip_x = t["flip_x"].as<bool>();
            tier.flip_y = t["flip_y"].as<bool>();

            // Use flat layer naming instead of nested "metal_layers"
            tier.min_metal_layer = t["min_metal_layer"].as<std::string>();
            tier.max_metal_layer = t["max_metal_layer"].as<std::string>();

            // Same for power nets
            tier.vdd_net = t["vdd_net"].as<std::string>();
            tier.gnd_net = t["gnd_net"].as<std::string>();
            tiers.push_back(tier);
        }
        cfg.setTiers(tiers);
    }

    // Integration Stack
    if (root["integration_stack"]) {
        std::vector<IntegrationLink> links;
        for (const auto& l : root["integration_stack"]) {
            IntegrationLink link;
            link.from_tier = l["from_tier"].as<std::string>();
            link.to_tier = l["to_tier"].as<std::string>();
            link.integration_type = l["integration_type"].as<std::string>();
            link.bonding_mode = l["bonding_mode"].as<std::string>();
            link.bonding_interface_layer = l["bonding_interface_layer"].as<std::string>();

            if (l["interconnects"]) {
                for (const auto& ic : l["interconnects"]) {
                    InterconnectInfo info;
                    info.type = ic["type"].as<std::string>();
                    info.name = ic["name"].as<std::string>();
                    if (ic["lef_path"])
                        info.lef_path = ic["lef_path"].as<std::string>();
                    else
                        logger->warn(utl::ODB, 2009, "Interconnect '{}' missing lef_path; skipping.", info.name);

                    // --- Parse new enclosure fields ---
                    if (ic["bottom_enclosure"]) {
                        auto be = ic["bottom_enclosure"];
                        info.bottom_enclosure.first = be[0].as<double>();
                        info.bottom_enclosure.second = be[1].as<double>();
                    }

                    if (ic["top_enclosure"]) {
                        auto te = ic["top_enclosure"];
                        info.top_enclosure.first = te[0].as<double>();
                        info.top_enclosure.second = te[1].as<double>();
                    }
                    link.interconnects.push_back(info);
                }
            }
            links.push_back(link);
        }
        cfg.setIntegrationStack(links);
    }

    // Thermal conductivity
    if (root["thermal"]["conductivity"]) {
        std::map<std::string, double> tc;
        for (auto it = root["thermal"]["conductivity"].begin();
             it != root["thermal"]["conductivity"].end(); ++it) {
            tc[it->first.as<std::string>()] = it->second.as<double>();
        }
        cfg.setThermalConductivity(tc);
    }

    return cfg;
}

// ---------------------------------------------------------
// Summary
// ---------------------------------------------------------
void Config_3DPDK::printSummary() const
{
    std::cout << "3D PDK Configuration Summary:\n";
    std::cout << "  Workspace: " << workspace_dir_ << "\n";
    std::cout << "  Integration Mode: " << integration_mode_ << "\n";
    std::cout << "  Tiers: " << tiers_.size() << "\n";
    for (const auto& t : tiers_) {
        std::cout << "    - " << t.name << " (" << t.role << ")\n";
    }
    std::cout << "  Integration Links: " << integration_stack_.size() << "\n";
    for (const auto& link : integration_stack_) {
        std::cout << "    " << link.from_tier << " → " << link.to_tier
                  << " [" << link.integration_type << "]"
                  << " via " << link.bonding_mode << "\n";
    }
}

}  // namespace odb
