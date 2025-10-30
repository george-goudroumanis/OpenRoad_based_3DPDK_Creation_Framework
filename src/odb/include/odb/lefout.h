// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <cstdint>
#include <map>
#include <ostream>
#include <sstream>
#include <string>
#include <unordered_map>

#include "boost/polygon/polygon.hpp"
#include "odb/db.h"
#include "odb/dbObject.h"
#include "odb/dbSet.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"
#include "odb/odb.h"
#include "odb/3DPDKconfig.h"
#include "utl/Logger.h"

namespace odb {

class dbTech;
class dbTechLayer;
class dbTechVia;
class dbLib;
class dbMaster;
class dbMTerm;
class dbDatabase;
class dbSite;
class dbTechSameNetRule;
class dbTechNonDefaultRule;
class dbTechLayerRule;
class dbTechViaRule;
class dbTechViaGenerateRule;
class dbProperty;

class lefout
{
  bool _use_master_ids;
  bool _use_alias;
  bool _write_marked_masters;
  double _dist_factor;
  double _area_factor;
  utl::Logger* logger_;
  int bloat_factor_;
  bool bloat_occupied_layers_;

  odb::Config_3DPDK* config_3D_pdk_;

  template <typename GenericBox>
  void writeBoxes(dbBlock* block, dbSet<GenericBox>& boxes, const char* indent);

  using ObstructionMap
      = std::map<dbTechLayer*, boost::polygon::polygon_90_set_data<int>>;

  void writeTechBody(dbTech* tech);
  void writeLayer(dbTechLayer* layer);
  void writeVia(dbTechVia* via);
  void writeBlockVia(dbBlock* db_block, dbVia* via);
  void writeHeader(dbLib* lib);
  void writeHeader(dbBlock* db_block);
  void writeLibBody(dbLib* lib);
  void writeMaster(dbMaster* master);
  void writeMTerm(dbMTerm* mterm);
  void writeSite(dbSite* site);
  void writeViaMap(dbTech* tech, bool use_via_cut_class);
  void writeNonDefaultRule(dbTech* tech, dbTechNonDefaultRule* rule);
  void writeLayerRule(dbTechLayerRule* rule);
  void writeSameNetRule(dbTechSameNetRule* rule);
  void writeTechViaRule(dbTechViaRule* rule);
  void writeTechViaGenerateRule(dbTechViaGenerateRule* rule);
  void writePropertyDefinition(dbProperty* prop);
  void writePropertyDefinitions(dbLib* lib);
  void writeVersion(const std::string& version);
  void writeNameCaseSensitive(dbOnOffType on_off_type);
  void writeBusBitChars(char left_bus_delimiter, char right_bus_delimiter);
  void writeUnits(int database_units);
  void writeDividerChar(char hier_delimiter);
  void writeObstructions(dbBlock* db_block);
  void getObstructions(dbBlock* db_block, ObstructionMap& obstructions) const;
  void writeBox(const std::string& indent, dbBox* box);
  void writePolygon(const std::string& indent, dbPolygon* polygon);
  void writeRect(const std::string& indent,
                 const boost::polygon::rectangle_data<int>& rect);
  void findInstsObstructions(ObstructionMap& obstructions,
                             dbBlock* db_block) const;
  void findWireLayerObstructions(ObstructionMap& obstructions,
                                 dbNet* net) const;
  void findSWireLayerObstructions(ObstructionMap& obstructions,
                                  dbNet* net) const;
  void findLayerViaObstructions(ObstructionMap& obstructions,
                                dbSBox* box) const;
  void writeBlock(dbBlock* db_block);
  void writePins(dbBlock* db_block);
  void writePowerPins(dbBlock* db_block);
  void writeBlockTerms(dbBlock* db_block);

  inline void writeObjectPropertyDefinitions(
      dbObject* obj,
      std::unordered_map<std::string, int16_t>& propertiesMap);

  int determineBloat(dbTechLayer* layer) const;
  void insertObstruction(dbTechLayer* layer,
                         const Rect& rect,
                         ObstructionMap& obstructions) const;
  void insertObstruction(dbBox* box, ObstructionMap& obstructions) const;

  template <typename GenericBox>
  void write3DBoxes(dbBlock* block, dbSet<GenericBox>& boxes, const char* indent, int tier);
  void write3DMTerm(dbMTerm* mterm, int tier);
  void write3DObstructions(dbBlock* db_block, int tier);
  void write3DBlockVia(dbBlock* db_block, dbVia* via, int tier);
  void write3DBlock(dbBlock* db_block, int tier);
  void write3DPins(dbBlock* db_block, int tier);
  void write3DPowerPins(dbBlock* db_block, int tier);
  void write3DBlockTerms(dbBlock* db_block, int tier);
  void write3DNonDefaultRule(dbTech* tech, dbTechNonDefaultRule* rule, int tier);
  void write3DLayerRule(dbTechLayerRule* rule, int tier);
  void write3DSameNetRule(dbTechSameNetRule* rule, int tier);
  void write3DTechViaRule(dbTechViaRule* rule, int tier);
  void write3DTechViaGenerateRule(dbTechViaGenerateRule* rule, int tier);
  void write3DVia(dbTechVia* via, int tier);
  void write3DLayer(dbTechLayer* layer, int tier);
  void write3DMaster(dbMaster* master, int tier);
  bool isValidIntegrationTransition(const std::string& previous, const std::string& current);
  dbTech* findTechForTier(const std::vector<dbTech*>& techs, const Config_3DPDK::TierInfo& tier);
  void writeTierLayers(dbTech* tier_tech, const Config_3DPDK::TierInfo& tier, const std::vector<dbTechLayer*>& tier_layers, size_t tier_idx, const std::string& integration_type);
  void writeGeneralTechInfo(dbTech* tech);
  void collectTierLayers(dbTech* tech, const Config_3DPDK::TierInfo& tier,std::vector<dbTechLayer*>& tier_layers);
  std::vector<dbTechLayer*> synthesizeMIVLayer(size_t tier_idx, odb::dbSet<odb::dbTech> MIV_tech);
  void writeTierWithIntegration(size_t tier_idx, const std::vector<dbTechLayer*>& tier_layers, dbTechLayer* miv_layer, const std::string& integration_type);
  void writeViasAndRules(int tier_idx, dbTech* tech, std::vector<dbTechLayer*> miv_table, std::vector<size_t> miv_indexes);
  void writeMakeTracksFile(const std::string& filename, const std::vector<odb::dbTech*>& techs);

 public:
  double lefdist(int value) { return value * _dist_factor; }
  double lefarea(int value) { return value * _area_factor; }

  lefout(utl::Logger* logger, std::ostream& out) : _out(out)
  {
    _write_marked_masters = _use_alias = _use_master_ids = false;
    _dist_factor = 0.001;
    _area_factor = 0.000001;
    logger_ = logger;
    bloat_factor_ = 10;
    bloat_occupied_layers_ = false;
  }

  void setWriteMarkedMasters(bool value) { _write_marked_masters = value; }
  void setUseLayerAlias(bool value) { _use_alias = value; }
  void setUseMasterIds(bool value) { _use_master_ids = value; }
  void setBloatFactor(int value) { bloat_factor_ = value; }
  void setBloatOccupiedLayers(bool value) { bloat_occupied_layers_ = value; }

  void writeTech(dbTech* tech);
  void writeLib(dbLib* lib);
  void writeTechAndLib(dbLib* lib);
  void writeAbstractLef(dbBlock* db_block);

  void write3DTechBody(const std::vector<dbTech*>& techs, odb::dbSet<odb::dbTech> MIV_tech);
  void write3DLibBody(dbLib* lib, int tier_index);
  void setConfig(odb::Config_3DPDK* config);
  void write3DTech(const std::vector<dbTech*>& techs, odb::dbSet<odb::dbTech> MIV_tech);
  void write3DLib(dbLib* lib, int tier_index);
  void write3DTechAndLib(std::vector<dbTech*>& techs, odb::dbSet<odb::dbTech> MIV_tech, std::vector<dbLib*>& libs);
  void write3DLEFs(const char* filename, odb::Config_3DPDK config);

  std::ostream& out() { return _out; }

 protected:
  std::ostream& _out;
};
}  // namespace odb
