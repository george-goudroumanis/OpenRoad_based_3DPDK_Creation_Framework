// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "odb/lefout.h"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include "boost/polygon/polygon.hpp"
#include "odb/db.h"
#include "odb/dbObject.h"
#include "odb/dbSet.h"
#include "odb/dbShape.h"
#include "odb/dbTransform.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"
#include "spdlog/fmt/ostr.h"
#include "utl/scope.h"
#include "utl/Logger.h"
#include "utl/MakeLogger.h"

using namespace boost::polygon::operators;
using namespace odb;

int lefout::determineBloat(dbTechLayer* layer) const
{
  int bloat = 0;

  const int pitch = layer->getPitch();
  if (pitch != 0) {
    bloat = pitch;
  } else {
    bloat = 2 * layer->getSpacing();
  }
  return bloat_factor_ * bloat;
}

void lefout::insertObstruction(dbBox* box, ObstructionMap& obstructions) const
{
  insertObstruction(box->getTechLayer(), box->getBox(), obstructions);
}

void lefout::insertObstruction(dbTechLayer* layer,
                               const Rect& rect,
                               ObstructionMap& obstructions) const
{
  if (layer->getType() == odb::dbTechLayerType::CUT) {
    return;
  }

  const int bloat = determineBloat(layer);
  boost::polygon::polygon_90_set_data<int> poly;
  poly = boost::polygon::rectangle_data<int>{
      rect.xMax(), rect.yMax(), rect.xMin(), rect.yMin()};
  obstructions[layer] += poly.bloat(bloat, bloat, bloat, bloat);
}

void lefout::writeVersion(const std::string& version)
{
  fmt::print(_out, "VERSION {} ;\n", version);
}

template <typename GenericBox>
void lefout::writeBoxes(dbBlock* block,
                        dbSet<GenericBox>& boxes,
                        const char* indent)
{
  dbTechLayer* cur_layer = nullptr;

  for (GenericBox* generic_box : boxes) {
    if (generic_box == nullptr) {
      continue;
    }

    dbBox* box = generic_box;
    dbTechLayer* layer = box->getTechLayer();

    // Checks if the box is either a tech via or a block via.
    if (box->getTechVia() || box->getBlockVia()) {
      std::string via_name;
      if (box->getTechVia()) {
        via_name = box->getTechVia()->getName();
      }
      if (box->getBlockVia()) {
        via_name = block->getName() + "_" + box->getBlockVia()->getName();
      }

      int x, y;
      box->getViaXY(x, y);
      fmt::print(_out,
                 "{}VIA {:.11g} {:.11g} {} ;\n",
                 indent,
                 lefdist(x),
                 lefdist(y),
                 via_name.c_str());
      cur_layer = nullptr;
    } else {
      std::string layer_name;
      if (_use_alias && layer->hasAlias()) {
        layer_name = layer->getAlias();
      } else {
        layer_name = layer->getName();
      }

      if (cur_layer != layer) {
        fmt::print(_out, "{}LAYER {} ;\n", indent, layer_name.c_str());
        cur_layer = layer;
      }

      writeBox(indent, box);
    }
  }
}

template <>
void lefout::writeBoxes(dbBlock* block,
                        dbSet<dbPolygon>& boxes,
                        const char* indent)
{
  dbTechLayer* cur_layer = nullptr;

  for (dbPolygon* box : boxes) {
    if (box == nullptr) {
      continue;
    }

    dbTechLayer* layer = box->getTechLayer();

    std::string layer_name;
    if (_use_alias && layer->hasAlias()) {
      layer_name = layer->getAlias();
    } else {
      layer_name = layer->getName();
    }

    if (cur_layer != layer) {
      fmt::print(_out, "{}LAYER {} ;\n", indent, layer_name.c_str());
      cur_layer = layer;
    }

    writePolygon(indent, box);
  }
}

void lefout::writeBox(const std::string& indent, dbBox* box)
{
  int x1 = box->xMin();
  int y1 = box->yMin();
  int x2 = box->xMax();
  int y2 = box->yMax();

  fmt::print(_out,
             "{}  RECT  {:.11g} {:.11g} {:.11g} {:.11g} ;\n",
             indent.c_str(),
             lefdist(x1),
             lefdist(y1),
             lefdist(x2),
             lefdist(y2));
}

void lefout::writePolygon(const std::string& indent, dbPolygon* polygon)
{
  fmt::print(_out, "{}  POLYGON  ", indent.c_str());

  for (const Point& pt : polygon->getPolygon().getPoints()) {
    int x = pt.x();
    int y = pt.y();
    fmt::print(_out, "{:.11g} {:.11g} ", lefdist(x), lefdist(y));
  }

  fmt::print(_out, ";\n");
}

void lefout::writeRect(const std::string& indent,
                       const boost::polygon::rectangle_data<int>& rect)
{
  int x1 = boost::polygon::xl(rect);
  int y1 = boost::polygon::yl(rect);
  int x2 = boost::polygon::xh(rect);
  int y2 = boost::polygon::yh(rect);

  fmt::print(_out,
             "{}  RECT  {:.11g} {:.11g} {:.11g} {:.11g} ;\n",
             indent.c_str(),
             lefdist(x1),
             lefdist(y1),
             lefdist(x2),
             lefdist(y2));
}

void lefout::writeHeader(dbBlock* db_block)
{
  char left_bus_delimiter = 0;
  char right_bus_delimiter = 0;
  char hier_delimiter = db_block->getHierarchyDelimiter();

  db_block->getBusDelimiters(left_bus_delimiter, right_bus_delimiter);

  if (left_bus_delimiter == 0) {
    left_bus_delimiter = '[';
  }

  if (right_bus_delimiter == 0) {
    right_bus_delimiter = ']';
  }

  if (hier_delimiter == 0) {
    hier_delimiter = '|';
  }

  writeVersion("5.8");
  writeBusBitChars(left_bus_delimiter, right_bus_delimiter);
  writeDividerChar(hier_delimiter);
  writeUnits(/*database_units = */ db_block->getDbUnitsPerMicron());
}

void lefout::writeObstructions(dbBlock* db_block)
{
  ObstructionMap obstructions;
  getObstructions(db_block, obstructions);

  fmt::print(_out, "{}", "  OBS\n");
  dbBox* block_bounding_box = db_block->getBBox();
  for (const auto& [tech_layer, polySet] : obstructions) {
    fmt::print(_out, "    LAYER {} ;\n", tech_layer->getName().c_str());

    if (bloat_occupied_layers_) {
      writeBox("   ", block_bounding_box);
    } else {
      const int bloat = determineBloat(tech_layer);
      boost::polygon::polygon_90_set_data<int> shrink_poly = polySet;
      shrink_poly.shrink2(bloat, bloat, bloat, bloat);

      // Decompose the polygon set to rectanges in non-preferred direction
      std::vector<boost::polygon::rectangle_data<int>> rects;
      if (tech_layer->getDirection() == odb::dbTechLayerDir::HORIZONTAL) {
        shrink_poly.get_rectangles(rects, boost::polygon::VERTICAL);
      } else if (tech_layer->getDirection() == odb::dbTechLayerDir::VERTICAL) {
        shrink_poly.get_rectangles(rects, boost::polygon::HORIZONTAL);
      } else if (tech_layer->getDirection() == odb::dbTechLayerDir::NONE) {
        shrink_poly.get_rectangles(rects);
      }

      for (const auto& rect : rects) {
        writeRect("   ", rect);
      }
    }
  }
  fmt::print(_out, "  END\n");
}

void lefout::getObstructions(dbBlock* db_block,
                             ObstructionMap& obstructions) const
{
  for (dbObstruction* obs : db_block->getObstructions()) {
    insertObstruction(obs->getBBox(), obstructions);
  }

  findInstsObstructions(obstructions, db_block);

  for (dbNet* net : db_block->getNets()) {
    findSWireLayerObstructions(obstructions, net);
    findWireLayerObstructions(obstructions, net);
  }
}

void lefout::findInstsObstructions(ObstructionMap& obstructions,
                                   dbBlock* db_block) const
{  // Find all insts obsturctions and Iterms

  for (auto* inst : db_block->getInsts()) {
    const dbTransform trans = inst->getTransform();

    // Add insts obstructions
    for (auto* obs : inst->getMaster()->getObstructions()) {
      Rect obs_rect = obs->getBox();
      trans.apply(obs_rect);
      insertObstruction(obs->getTechLayer(), obs_rect, obstructions);
    }

    // Add inst Iterms to obstructions
    for (auto* iterm : inst->getITerms()) {
      dbShape shape;
      dbITermShapeItr iterm_shape_itr(/* expand_vias */ true);
      for (iterm_shape_itr.begin(iterm); iterm_shape_itr.next(shape);) {
        insertObstruction(shape.getTechLayer(), shape.getBox(), obstructions);
      }
    }
  }
}

void lefout::findSWireLayerObstructions(ObstructionMap& obstructions,
                                        dbNet* net) const
{  // Find all layers where an swire exists
  for (dbSWire* swire : net->getSWires()) {
    for (dbSBox* box : swire->getWires()) {
      if (box->isVia()) {
        // In certain power grid arrangements there may be a metal layer that
        // isn't directly used for straps or stripes just punching vias through.
        // In these cases the metal layer should still be blocked even though
        // we can't find any metal wires on the layer.
        // https://github.com/The-OpenROAD-Project/OpenROAD/pull/725#discussion_r669927312
        findLayerViaObstructions(obstructions, box);
      } else {
        insertObstruction(box, obstructions);
      }
    }
  }
}

void lefout::findLayerViaObstructions(ObstructionMap& obstructions,
                                      dbSBox* box) const
{
  std::vector<dbShape> via_shapes;
  box->getViaBoxes(via_shapes);
  for (dbShape db_shape : via_shapes) {
    if (db_shape.isViaBox()) {
      continue;
    }
    insertObstruction(db_shape.getTechLayer(), db_shape.getBox(), obstructions);
  }
}

void lefout::findWireLayerObstructions(ObstructionMap& obstructions,
                                       dbNet* net) const
{
  // Find all metal layers where a wire exists.
  dbWire* wire = net->getWire();

  if (wire == nullptr) {
    return;
  }

  dbWireShapeItr wire_shape_itr;
  dbShape shape;

  for (wire_shape_itr.begin(wire); wire_shape_itr.next(shape);) {
    if (shape.isVia()) {
      continue;
    }
    insertObstruction(shape.getTechLayer(), shape.getBox(), obstructions);
  }
}

void lefout::writeHeader(dbLib* lib)
{
  dbTech* tech = lib->getTech();

  char left_bus_delimiter = 0;
  char right_bus_delimiter = 0;
  char hier_delimiter = lib->getHierarchyDelimiter();

  lib->getBusDelimiters(left_bus_delimiter, right_bus_delimiter);

  if (left_bus_delimiter == 0) {
    left_bus_delimiter = '[';
  }

  if (right_bus_delimiter == 0) {
    right_bus_delimiter = ']';
  }

  if (hier_delimiter == 0) {
    hier_delimiter = '|';
  }

  writeVersion(tech->getLefVersionStr());
  writeNameCaseSensitive(tech->getNamesCaseSensitive());
  writeBusBitChars(left_bus_delimiter, right_bus_delimiter);
  writeDividerChar(hier_delimiter);
  writePropertyDefinitions(lib);

  if (lib->getLefUnits()) {
    writeUnits(lib->getLefUnits());
  }
}

void lefout::writeDividerChar(char hier_delimiter)
{
  fmt::print(_out, "DIVIDERCHAR \"{}\" ;\n", hier_delimiter);
}

void lefout::writeUnits(int database_units)
{
  fmt::print(_out, "{}", "UNITS\n");
  fmt::print(_out, "    DATABASE MICRONS {} ;\n", database_units);
  fmt::print(_out, "{}", "END UNITS\n");
}

void lefout::writeBusBitChars(char left_bus_delimiter, char right_bus_delimiter)
{
  fmt::print(_out,
             "BUSBITCHARS \"{}{}\" ;\n",
             left_bus_delimiter,
             right_bus_delimiter);
}

void lefout::writeNameCaseSensitive(const dbOnOffType on_off_type)
{
  fmt::print(_out, "NAMESCASESENSITIVE {} ;\n", on_off_type.getString());
}

void lefout::writeBlockVia(dbBlock* db_block, dbVia* via)
{
  std::string name = db_block->getName() + "_" + via->getName();

  if (via->isDefault()) {
    fmt::print(_out, "\nVIA {} DEFAULT\n", name.c_str());
  } else {
    fmt::print(_out, "\nVIA {}\n", name.c_str());
  }

  dbTechViaGenerateRule* rule = via->getViaGenerateRule();

  if (rule == nullptr) {
    dbSet<dbBox> boxes = via->getBoxes();
    writeBoxes(db_block, boxes, "    ");
  } else {
    std::string rname = rule->getName();
    fmt::print(_out, "  VIARULE {} ;\n", rname.c_str());

    const dbViaParams P = via->getViaParams();

    fmt::print(_out,
               "  CUTSIZE {:.11g} {:.11g} ;\n",
               lefdist(P.getXCutSize()),
               lefdist(P.getYCutSize()));
    std::string top = P.getTopLayer()->getName();
    std::string bot = P.getBottomLayer()->getName();
    std::string cut = P.getCutLayer()->getName();
    fmt::print(
        _out, "  LAYERS {} {} {} ;\n", bot.c_str(), cut.c_str(), top.c_str());
    fmt::print(_out,
               "  CUTSPACING {:.11g} {:.11g} ;\n",
               lefdist(P.getXCutSpacing()),
               lefdist(P.getYCutSpacing()));
    fmt::print(_out,
               "  ENCLOSURE {:.11g} {:.11g} {:.11g} {:.11g} ;\n",
               lefdist(P.getXBottomEnclosure()),
               lefdist(P.getYBottomEnclosure()),
               lefdist(P.getXTopEnclosure()),
               lefdist(P.getYTopEnclosure()));

    if ((P.getNumCutRows() != 1) || (P.getNumCutCols() != 1)) {
      fmt::print(
          _out, "  ROWCOL {} {} ;\n", P.getNumCutRows(), P.getNumCutCols());
    }

    if ((P.getXOrigin() != 0) || (P.getYOrigin() != 0)) {
      fmt::print(_out,
                 "  ORIGIN {:.11g} {:.11g} ;\n",
                 lefdist(P.getXOrigin()),
                 lefdist(P.getYOrigin()));
    }

    if ((P.getXTopOffset() != 0) || (P.getYTopOffset() != 0)
        || (P.getXBottomOffset() != 0) || (P.getYBottomOffset() != 0)) {
      fmt::print(_out,
                 "  OFFSET {:.11g} {:.11g} {:.11g} {:.11g} ;\n",
                 lefdist(P.getXBottomOffset()),
                 lefdist(P.getYBottomOffset()),
                 lefdist(P.getXTopOffset()),
                 lefdist(P.getYTopOffset()));
    }

    std::string pname = via->getPattern();
    if (strcmp(pname.c_str(), "") != 0) {
      fmt::print(_out, "  PATTERNNAME {} ;\n", pname.c_str());
    }
  }

  fmt::print(_out, "END {}\n", name.c_str());
}

void lefout::writeBlock(dbBlock* db_block)
{
  Rect die_area = db_block->getDieArea();
  double size_x = lefdist(die_area.xMax());
  double size_y = lefdist(die_area.yMax());

  for (auto via : db_block->getVias()) {
    writeBlockVia(db_block, via);
  }

  fmt::print(_out, "\nMACRO {}\n", db_block->getName().c_str());
  fmt::print(_out, "  FOREIGN {} 0 0 ;\n", db_block->getName().c_str());
  fmt::print(_out, "  CLASS BLOCK ;\n");
  fmt::print(_out, "  SIZE {:.11g} BY {:.11g} ;\n", size_x, size_y);
  writePins(db_block);
  writeObstructions(db_block);
  fmt::print(_out, "END {}\n", db_block->getName().c_str());
}

void lefout::writePins(dbBlock* db_block)
{
  writePowerPins(db_block);
  writeBlockTerms(db_block);
}

void lefout::writeBlockTerms(dbBlock* db_block)
{
  for (dbBTerm* b_term : db_block->getBTerms()) {
    fmt::print(_out, "  PIN {}\n", b_term->getName().c_str());
    fmt::print(_out, "    DIRECTION {} ;\n", b_term->getIoType().getString());

    std::string sig_type = "SIGNAL";
    switch (b_term->getSigType().getValue()) {
      case odb::dbSigType::SIGNAL:
      case odb::dbSigType::SCAN:
      case odb::dbSigType::TIEOFF:
      case odb::dbSigType::RESET:
        // LEF Pins can only be pin : [USE { SIGNAL | ANALOG | POWER | GROUND |
        // CLOCK } ;] whereas nets can be net: [+ USE {ANALOG | CLOCK | GROUND |
        // POWER | RESET | SCAN | SIGNAL | TIEOFF}] BTerms get their sigType
        // from the net. So we map other sigtypes to SIGNAL in exported LEFs
        sig_type = "SIGNAL";
        break;
      case odb::dbSigType::ANALOG:
        sig_type = "ANALOG";
        break;
      case odb::dbSigType::POWER:
        sig_type = "POWER";
        break;
      case odb::dbSigType::GROUND:
        sig_type = "GROUND";
        break;
      case odb::dbSigType::CLOCK:
        sig_type = "CLOCK";
        break;
    }
    fmt::print(_out, "    USE {} ;\n", sig_type);

    for (dbBPin* db_b_pin : b_term->getBPins()) {
      fmt::print(_out, "{}", "    PORT\n");
      dbSet<dbBox> term_pins = db_b_pin->getBoxes();
      writeBoxes(db_block, term_pins, "      ");
      fmt::print(_out, "{}", "    END\n");
    }
    fmt::print(_out, "  END {}\n", b_term->getName().c_str());
  }
}

void lefout::writePowerPins(dbBlock* db_block)
{  // Power Ground.
  for (dbNet* net : db_block->getNets()) {
    if (!net->getSigType().isSupply()) {
      continue;
    }
    if (net->get1stBTerm() != nullptr) {
      // net already has pins that will be added
      continue;
    }
    fmt::print(_out, "  PIN {}\n", net->getName().c_str());
    fmt::print(_out, "    USE {} ;\n", net->getSigType().getString());
    fmt::print(
        _out, "    DIRECTION {} ;\n", dbIoType(dbIoType::INOUT).getString());
    for (dbSWire* special_wire : net->getSWires()) {
      fmt::print(_out, "    PORT\n");
      dbSet<dbSBox> wires = special_wire->getWires();
      writeBoxes(db_block, wires, /*indent=*/"      ");
      fmt::print(_out, "    END\n");
    }
    fmt::print(_out, "  END {}\n", net->getName().c_str());
  }
}

void lefout::writeTechBody(dbTech* tech)
{
  assert(tech);

  if (tech->hasNoWireExtAtPin()) {
    fmt::print(_out,
               "NOWIREEXTENSIONATPIN {} ;\n",
               tech->getNoWireExtAtPin().getString());
  }

  if (tech->hasClearanceMeasure()) {
    fmt::print(_out,
               "CLEARANCEMEASURE {} ;\n",
               tech->getClearanceMeasure().getString());
  }

  if (tech->hasUseMinSpacingObs()) {
    fmt::print(_out,
               "USEMINSPACING OBS {} ;\n",
               tech->getUseMinSpacingObs().getString());
  }

  if (tech->hasUseMinSpacingPin()) {
    fmt::print(_out,
               "USEMINSPACING PIN {} ;\n",
               tech->getUseMinSpacingPin().getString());
  }

  if (tech->hasManufacturingGrid()) {
    fmt::print(_out,
               "MANUFACTURINGGRID {:.11g} ;\n",
               lefdist(tech->getManufacturingGrid()));
  }

  dbSet<dbTechLayer> layers = tech->getLayers();
  dbSet<dbTechLayer>::iterator litr;

  for (litr = layers.begin(); litr != layers.end(); ++litr) {
    dbTechLayer* layer = *litr;
    writeLayer(layer);
  }

  writeViaMap(tech, false);
  writeViaMap(tech, true);

  // VIA's not using generate rule and not default
  dbSet<dbTechVia> vias = tech->getVias();
  dbSet<dbTechVia>::iterator vitr;

  for (vitr = vias.begin(); vitr != vias.end(); ++vitr) {
    dbTechVia* via = *vitr;

    if (via->getNonDefaultRule() == nullptr) {
      if (via->getViaGenerateRule() == nullptr) {
        writeVia(via);
      }
    }
  }

  dbSet<dbTechViaRule> via_rules = tech->getViaRules();
  dbSet<dbTechViaRule>::iterator vritr;

  for (vritr = via_rules.begin(); vritr != via_rules.end(); ++vritr) {
    dbTechViaRule* rule = *vritr;
    writeTechViaRule(rule);
  }

  dbSet<dbTechViaGenerateRule> via_gen_rules = tech->getViaGenerateRules();
  dbSet<dbTechViaGenerateRule>::iterator vgritr;

  for (vgritr = via_gen_rules.begin(); vgritr != via_gen_rules.end();
       ++vgritr) {
    dbTechViaGenerateRule* rule = *vgritr;
    writeTechViaGenerateRule(rule);
  }

  // VIA's using generate rule
  vias = tech->getVias();

  for (vitr = vias.begin(); vitr != vias.end(); ++vitr) {
    dbTechVia* via = *vitr;

    if (via->getNonDefaultRule() == nullptr) {
      if (via->getViaGenerateRule() != nullptr) {
        writeVia(via);
      }
    }
  }

  std::vector<dbTechSameNetRule*> srules;
  tech->getSameNetRules(srules);

  if (srules.begin() != srules.end()) {
    fmt::print(_out, "\nSPACING\n");

    std::vector<dbTechSameNetRule*>::iterator sritr;
    for (sritr = srules.begin(); sritr != srules.end(); ++sritr) {
      writeSameNetRule(*sritr);
    }

    fmt::print(_out, "\nEND SPACING\n");
  }

  dbSet<dbTechNonDefaultRule> rules = tech->getNonDefaultRules();
  dbSet<dbTechNonDefaultRule>::iterator ritr;

  for (ritr = rules.begin(); ritr != rules.end(); ++ritr) {
    dbTechNonDefaultRule* rule = *ritr;
    writeNonDefaultRule(tech, rule);
  }
}

void lefout::writeViaMap(dbTech* tech, const bool use_via_cut_class)
{
  auto via_map_set = tech->getMetalWidthViaMap();
  bool found = false;
  for (auto via_map : via_map_set) {
    if (via_map->isViaCutClass() == use_via_cut_class) {
      found = true;
      break;
    }
  }
  if (!found) {
    return;
  }
  fmt::print(_out, "PROPERTYDEFINITIONS\n");
  fmt::print(_out, " LIBRARY LEF58_METALWIDTHVIAMAP STRING\n");
  fmt::print(_out, "  \"METALWIDTHVIAMAP\n");
  if (use_via_cut_class) {
    fmt::print(_out, "   USEVIACUTCLASS\n");
  }
  for (auto via_map : via_map_set) {
    if (via_map->isViaCutClass() != use_via_cut_class) {
      continue;
    }
    if (via_map->getBelowLayerWidthLow() == via_map->getBelowLayerWidthHigh()
        && via_map->getAboveLayerWidthLow()
               == via_map->getAboveLayerWidthHigh()) {
      fmt::print(_out,
                 "   VIA {} {} {} {} {}\n",
                 via_map->getCutLayer()->getName(),
                 lefdist(via_map->getBelowLayerWidthLow()),
                 lefdist(via_map->getAboveLayerWidthLow()),
                 via_map->getViaName(),
                 via_map->isPgVia() ? "PGVIA" : "");
    } else {
      fmt::print(_out,
                 "   VIA {} {} {} {} {} {} {}\n",
                 via_map->getCutLayer()->getName(),
                 lefdist(via_map->getBelowLayerWidthLow()),
                 lefdist(via_map->getBelowLayerWidthHigh()),
                 lefdist(via_map->getAboveLayerWidthLow()),
                 lefdist(via_map->getAboveLayerWidthHigh()),
                 via_map->getViaName(),
                 via_map->isPgVia() ? "PGVIA" : "");
    }
  }
  fmt::print(_out, "   ;\n");
  fmt::print(_out, " \" ;\n");
  fmt::print(_out, "END PROPERTYDEFINITIONS\n");
}

void lefout::writeNonDefaultRule(dbTech* tech, dbTechNonDefaultRule* rule)
{
  std::string name = rule->getName();
  fmt::print(_out, "\nNONDEFAULTRULE {}\n", name.c_str());

  if (rule->getHardSpacing()) {
    fmt::print(_out, "{}", "HARDSPACING ;\n");
  }

  std::vector<dbTechLayerRule*> layer_rules;
  rule->getLayerRules(layer_rules);

  std::vector<dbTechLayerRule*>::iterator litr;
  for (litr = layer_rules.begin(); litr != layer_rules.end(); ++litr) {
    writeLayerRule(*litr);
  }

  std::vector<dbTechVia*> vias;
  rule->getVias(vias);

  std::vector<dbTechVia*>::iterator vitr;
  for (vitr = vias.begin(); vitr != vias.end(); ++vitr) {
    writeVia(*vitr);
  }

  std::vector<dbTechSameNetRule*> srules;
  rule->getSameNetRules(srules);

  if (srules.begin() != srules.end()) {
    fmt::print(_out, "\nSPACING\n");

    std::vector<dbTechSameNetRule*>::iterator sritr;
    for (sritr = srules.begin(); sritr != srules.end(); ++sritr) {
      writeSameNetRule(*sritr);
    }

    fmt::print(_out, "\nEND SPACING\n");
  }

  std::vector<dbTechVia*> use_vias;
  rule->getUseVias(use_vias);

  std::vector<dbTechVia*>::iterator uvitr;
  for (uvitr = use_vias.begin(); uvitr != use_vias.end(); ++uvitr) {
    dbTechVia* via = *uvitr;
    std::string vname = via->getName();
    fmt::print(_out, "USEVIA {} ;\n", vname.c_str());
  }

  std::vector<dbTechViaGenerateRule*> use_rules;
  rule->getUseViaRules(use_rules);

  std::vector<dbTechViaGenerateRule*>::iterator uvritr;
  for (uvritr = use_rules.begin(); uvritr != use_rules.end(); ++uvritr) {
    dbTechViaGenerateRule* rule = *uvritr;
    std::string rname = rule->getName();
    fmt::print(_out, "USEVIARULE {} ;\n", rname.c_str());
  }

  dbSet<dbTechLayer> layers = tech->getLayers();
  dbSet<dbTechLayer>::iterator layitr;

  for (layitr = layers.begin(); layitr != layers.end(); ++layitr) {
    dbTechLayer* layer = *layitr;
    int count;

    if (rule->getMinCuts(layer, count)) {
      std::string lname = layer->getName();
      fmt::print(_out, "MINCUTS {} {} ;\n", lname.c_str(), count);
    }
  }

  fmt::print(_out, "\nEND {}\n", name.c_str());
}

void lefout::writeLayerRule(dbTechLayerRule* rule)
{
  dbTechLayer* layer = rule->getLayer();
  std::string name;
  if (_use_alias && layer->hasAlias()) {
    name = layer->getAlias();
  } else {
    name = layer->getName();
  }
  fmt::print(_out, "\nLAYER {}\n", name.c_str());

  if (rule->getWidth()) {
    fmt::print(_out, "    WIDTH {:.11g} ;\n", lefdist(rule->getWidth()));
  }

  if (rule->getSpacing()) {
    fmt::print(_out, "    SPACING {:.11g} ;\n", lefdist(rule->getSpacing()));
  }

  if (rule->getWireExtension() != 0.0) {
    fmt::print(_out,
               "    WIREEXTENSION {:.11g} ;\n",
               lefdist(rule->getWireExtension()));
  }

  if (rule->getResistance() != 0.0) {
    fmt::print(
        _out, "    RESISTANCE RPERSQ {:.11g} ;\n", rule->getResistance());
  }

  if (rule->getCapacitance() != 0.0) {
    fmt::print(
        _out, "    CAPACITANCE CPERSQDIST {:.11g} ;\n", rule->getCapacitance());
  }

  if (rule->getEdgeCapacitance() != 0.0) {
    fmt::print(
        _out, "      EDGECAPACITANCE {:.11g} ;\n", rule->getEdgeCapacitance());
  }

  fmt::print(_out, "END {}\n", name.c_str());
}

void lefout::writeTechViaRule(dbTechViaRule* rule)
{
  std::string name = rule->getName();
  fmt::print(_out, "\nVIARULE {}\n", name.c_str());

  uint idx;

  for (idx = 0; idx < rule->getViaLayerRuleCount(); ++idx) {
    dbTechViaLayerRule* layrule = rule->getViaLayerRule(idx);
    dbTechLayer* layer = layrule->getLayer();
    std::string lname = layer->getName();
    fmt::print(_out, "    LAYER {} ;\n", lname.c_str());

    if (layrule->getDirection() == dbTechLayerDir::VERTICAL) {
      fmt::print(_out, "      DIRECTION VERTICAL ;\n");
    } else if (layrule->getDirection() == dbTechLayerDir::HORIZONTAL) {
      fmt::print(_out, "      DIRECTION HORIZONTAL ;\n");
    }

    if (layrule->hasWidth()) {
      int minW, maxW;
      layrule->getWidth(minW, maxW);
      fmt::print(_out,
                 "      WIDTH {:.11g} TO {:.11g} ;\n",
                 lefdist(minW),
                 lefdist(maxW));
    }
  }

  for (idx = 0; idx < rule->getViaCount(); ++idx) {
    dbTechVia* via = rule->getVia(idx);
    std::string vname = via->getName();
    fmt::print(_out, "    VIA {} ;\n", vname.c_str());
  }

  fmt::print(_out, "END {}\n", name.c_str());
}

void lefout::writeTechViaGenerateRule(dbTechViaGenerateRule* rule)
{
  std::string name = rule->getName();

  if (rule->isDefault()) {
    fmt::print(_out, "\nVIARULE {} GENERATE DEFAULT\n", name.c_str());
  } else {
    fmt::print(_out, "\nVIARULE {} GENERATE \n", name.c_str());
  }

  uint idx;

  for (idx = 0; idx < rule->getViaLayerRuleCount(); ++idx) {
    dbTechViaLayerRule* layrule = rule->getViaLayerRule(idx);
    dbTechLayer* layer = layrule->getLayer();
    std::string lname = layer->getName();
    fmt::print(_out, "    LAYER {} ;\n", lname.c_str());

    if (layrule->getDirection() == dbTechLayerDir::VERTICAL) {
      fmt::print(_out, "      DIRECTION VERTICAL ;\n");
    } else if (layrule->getDirection() == dbTechLayerDir::HORIZONTAL) {
      fmt::print(_out, "      DIRECTION HORIZONTAL ;\n");
    }

    if (layrule->hasOverhang()) {
      fmt::print(
          _out, "      OVERHANG {:.11g} ;\n", lefdist(layrule->getOverhang()));
    }

    if (layrule->hasMetalOverhang()) {
      fmt::print(_out,
                 "      METALOVERHANG {:.11g} ;\n",
                 lefdist(layrule->getMetalOverhang()));
    }

    if (layrule->hasEnclosure()) {
      int overhang1, overhang2;
      layrule->getEnclosure(overhang1, overhang2);
      fmt::print(_out,
                 "      ENCLOSURE {:.11g} {:.11g} ;\n",
                 lefdist(overhang1),
                 lefdist(overhang2));
    }

    if (layrule->hasWidth()) {
      int minW, maxW;
      layrule->getWidth(minW, maxW);
      fmt::print(_out,
                 "      WIDTH {:.11g} TO {:.11g} ;\n",
                 lefdist(minW),
                 lefdist(maxW));
    }

    if (layrule->hasRect()) {
      Rect r;
      layrule->getRect(r);
      fmt::print(_out,
                 "      RECT  {:.11g} {:.11g}  {:.11g} {:.11g}  ;\n",
                 lefdist(r.xMin()),
                 lefdist(r.yMin()),
                 lefdist(r.xMax()),
                 lefdist(r.yMax()));
    }

    if (layrule->hasSpacing()) {
      int spacing_x, spacing_y;
      layrule->getSpacing(spacing_x, spacing_y);
      fmt::print(_out,
                 "      SPACING {:.11g} BY {:.11g} ;\n",
                 lefdist(spacing_x),
                 lefdist(spacing_y));
    }

    if (layrule->hasResistance()) {
      fmt::print(
          _out, "      RESISTANCE {:.11g} ;\n", layrule->getResistance());
    }
  }

  fmt::print(_out, "END {}\n", name.c_str());
}

void lefout::writeSameNetRule(dbTechSameNetRule* rule)
{
  dbTechLayer* l1 = rule->getLayer1();
  dbTechLayer* l2 = rule->getLayer2();

  std::string n1;
  if (_use_alias && l1->hasAlias()) {
    n1 = l1->getAlias();
  } else {
    n1 = l1->getName();
  }

  std::string n2;
  if (_use_alias && l2->hasAlias()) {
    n2 = l2->getAlias();
  } else {
    n2 = l2->getName();
  }

  if (rule->getAllowStackedVias()) {
    fmt::print(_out,
               "  SAMENET {} {} {:.11g} STACK ;\n",
               n1.c_str(),
               n2.c_str(),
               lefdist(rule->getSpacing()));
  } else {
    fmt::print(_out,
               "  SAMENET {} {} {:.11g} ;\n",
               n1.c_str(),
               n2.c_str(),
               lefdist(rule->getSpacing()));
  }
}

void lefout::writeLayer(dbTechLayer* layer)
{
  std::string name;
  if (_use_alias && layer->hasAlias()) {
    name = layer->getAlias();
  } else {
    name = layer->getName();
  }

  fmt::print(_out, "\nLAYER {}\n", name.c_str());
  fmt::print(_out, "    TYPE {} ;\n", layer->getType().getString());

  if (layer->getNumMasks() > 1) {
    fmt::print(_out, "    MASK {} ;\n", layer->getNumMasks());
  }

  if (layer->getPitch()) {
    fmt::print(_out, "    PITCH {:.11g} ;\n", lefdist(layer->getPitch()));
  }

  if (layer->getWidth()) {
    fmt::print(_out, "    WIDTH {:.11g} ;\n", lefdist(layer->getWidth()));
  }

  if (layer->getWireExtension() != 0.0) {
    fmt::print(_out,
               "    WIREEXTENSION {:.11g} ;\n",
               lefdist(layer->getWireExtension()));
  }

  if (layer->hasArea()) {
    fmt::print(_out, "    AREA {:.11g} ;\n", layer->getArea());
  }

  uint thickness;
  if (layer->getThickness(thickness)) {
    fmt::print(_out, "    THICKNESS {:.11g} ;\n", lefdist(thickness));
  }

  if (layer->hasMaxWidth()) {
    fmt::print(_out, "    MAXWIDTH {:.11g} ;\n", lefdist(layer->getMaxWidth()));
  }

  if (layer->hasMinStep()) {
    fmt::print(_out, "    MINSTEP {:.11g} ;\n", lefdist(layer->getMinStep()));
  }

  if (layer->hasProtrusion()) {
    fmt::print(_out,
               "    PROTRUSIONWIDTH {:.11g}  LENGTH {:.11g}  WIDTH {:.11g} ;\n",
               lefdist(layer->getProtrusionWidth()),
               lefdist(layer->getProtrusionLength()),
               lefdist(layer->getProtrusionFromWidth()));
  }

  for (auto rule : layer->getV54SpacingRules()) {
    rule->writeLef(*this);
  }

  if (layer->hasV55SpacingRules()) {
    layer->printV55SpacingRules(*this);
    auto inf_rules = layer->getV55InfluenceRules();
    if (!inf_rules.empty()) {
      fmt::print(_out, "SPACINGTABLE INFLUENCE");
      for (auto rule : inf_rules) {
        rule->writeLef(*this);
      }
      fmt::print(_out, " ;\n");
    }
  }

  std::vector<dbTechMinCutRule*> cut_rules;
  std::vector<dbTechMinCutRule*>::const_iterator citr;
  if (layer->getMinimumCutRules(cut_rules)) {
    for (citr = cut_rules.begin(); citr != cut_rules.end(); citr++) {
      (*citr)->writeLef(*this);
    }
  }

  std::vector<dbTechMinEncRule*> enc_rules;
  std::vector<dbTechMinEncRule*>::const_iterator eitr;
  if (layer->getMinEnclosureRules(enc_rules)) {
    for (eitr = enc_rules.begin(); eitr != enc_rules.end(); eitr++) {
      (*eitr)->writeLef(*this);
    }
  }

  layer->writeAntennaRulesLef(*this);

  if (layer->getDirection() != dbTechLayerDir::NONE) {
    fmt::print(_out, "    DIRECTION {} ;\n", layer->getDirection().getString());
  }

  if (layer->getResistance() != 0.0) {
    if (layer->getType() == dbTechLayerType::CUT) {
      fmt::print(_out, "    RESISTANCE {:.11g} ;\n", layer->getResistance());
    } else {
      fmt::print(
          _out, "    RESISTANCE RPERSQ {:.11g} ;\n", layer->getResistance());
    }
  }

  if (layer->getCapacitance() != 0.0) {
    fmt::print(_out,
               "    CAPACITANCE CPERSQDIST {:.11g} ;\n",
               layer->getCapacitance());
  }

  if (layer->getEdgeCapacitance() != 0.0) {
    fmt::print(
        _out, "    EDGECAPACITANCE {:.11g} ;\n", layer->getEdgeCapacitance());
  }

  fmt::print(_out, "{}", dbProperty::writeProperties(layer));

  fmt::print(_out, "END {}\n", name.c_str());
}

void lefout::writeVia(dbTechVia* via)
{
  std::string name = via->getName();

  if (via->isDefault()) {
    fmt::print(_out, "\nVIA {} DEFAULT\n", name.c_str());
  } else {
    fmt::print(_out, "\nVIA {}\n", name.c_str());
  }

  if (via->isTopOfStack()) {
    fmt::print(_out, "    TOPOFSTACKONLY\n");
  }

  if (via->getResistance() != 0.0) {
    fmt::print(_out, "    RESISTANCE {:.11g} ;\n", via->getResistance());
  }

  dbTechViaGenerateRule* rule = via->getViaGenerateRule();

  if (rule == nullptr) {
    dbSet<dbBox> boxes = via->getBoxes();
    writeBoxes(nullptr, boxes, "    ");
  } else {
    std::string rname = rule->getName();
    fmt::print(_out, "\n    VIARULE {} \n", rname.c_str());

    const dbViaParams P = via->getViaParams();

    fmt::print(_out,
               " + CUTSIZE {:.11g} {:.11g}\n",
               lefdist(P.getXCutSize()),
               lefdist(P.getYCutSize()));
    std::string top = P.getTopLayer()->getName();
    std::string bot = P.getBottomLayer()->getName();
    std::string cut = P.getCutLayer()->getName();
    fmt::print(
        _out, " + LAYERS {} {} {}\n", bot.c_str(), cut.c_str(), top.c_str());
    fmt::print(_out,
               " + CUTSPACING {:.11g} {:.11g}\n",
               lefdist(P.getXCutSpacing()),
               lefdist(P.getYCutSpacing()));
    fmt::print(_out,
               " + ENCLOSURE {:.11g} {:.11g} {:.11g} {:.11g}\n",
               lefdist(P.getXBottomEnclosure()),
               lefdist(P.getYBottomEnclosure()),
               lefdist(P.getXTopEnclosure()),
               lefdist(P.getYTopEnclosure()));

    if ((P.getNumCutRows() != 1) || (P.getNumCutCols() != 1)) {
      fmt::print(
          _out, " + ROWCOL {} {}\n", P.getNumCutRows(), P.getNumCutCols());
    }

    if ((P.getXOrigin() != 0) || (P.getYOrigin() != 0)) {
      fmt::print(_out,
                 " + ORIGIN {:.11g} {:.11g}\n",
                 lefdist(P.getXOrigin()),
                 lefdist(P.getYOrigin()));
    }

    if ((P.getXTopOffset() != 0) || (P.getYTopOffset() != 0)
        || (P.getXBottomOffset() != 0) || (P.getYBottomOffset() != 0)) {
      fmt::print(_out,
                 " + OFFSET {:.11g} {:.11g} {:.11g} {:.11g}\n",
                 lefdist(P.getXBottomOffset()),
                 lefdist(P.getYBottomOffset()),
                 lefdist(P.getXTopOffset()),
                 lefdist(P.getYTopOffset()));
    }

    std::string pname = via->getPattern();
    if (strcmp(pname.c_str(), "") != 0) {
      fmt::print(_out, " + PATTERNNAME {}\n", pname.c_str());
    }
  }

  fmt::print(_out, "END {}\n", name.c_str());
}

void lefout::writeLibBody(dbLib* lib)
{
  dbSet<dbSite> sites = lib->getSites();
  dbSet<dbSite>::iterator site_itr;

  for (site_itr = sites.begin(); site_itr != sites.end(); ++site_itr) {
    dbSite* site = *site_itr;
    writeSite(site);
  }

  dbSet<dbMaster> masters = lib->getMasters();
  dbSet<dbMaster>::iterator master_itr;

  for (master_itr = masters.begin(); master_itr != masters.end();
       ++master_itr) {
    dbMaster* master = *master_itr;
    if (_write_marked_masters && !master->isMarked()) {
      continue;
    }
    writeMaster(master);
  }
}

void lefout::writeSite(dbSite* site)
{
  std::string n = site->getName();

  fmt::print(_out, "SITE {}\n", n.c_str());
  dbSiteClass sclass = site->getClass();
  fmt::print(_out, "    CLASS {} ;\n", sclass.getString());

  if (site->getSymmetryX() || site->getSymmetryY() || site->getSymmetryR90()) {
    fmt::print(_out, "{}", "    SYMMETRY");

    if (site->getSymmetryX()) {
      fmt::print(_out, "{}", " X");
    }

    if (site->getSymmetryY()) {
      fmt::print(_out, "{}", " Y");
    }

    if (site->getSymmetryR90()) {
      fmt::print(_out, "{}", " R90");
    }

    fmt::print(_out, "{}", " ;\n");
  }

  if (site->getWidth() || site->getHeight()) {
    fmt::print(_out,
               "    SIZE {:.11g} BY {:.11g} ;\n",
               lefdist(site->getWidth()),
               lefdist(site->getHeight()));
  }

  fmt::print(_out, "END {}\n", n.c_str());
}

void lefout::writeMaster(dbMaster* master)
{
  std::string name = master->getName();

  if (_use_master_ids) {
    fmt::print(_out,
               "\nMACRO M{}\n",
               static_cast<std::uint32_t>(master->getMasterId()));
  } else {
    fmt::print(_out, "\nMACRO {}\n", name.c_str());
  }

  fmt::print(_out, "    CLASS {} ;\n", master->getType().getString());

  const odb::Point origin = master->getOrigin();

  if (origin != Point()) {
    fmt::print(_out,
               "    ORIGIN {:.11g} {:.11g} ;\n",
               lefdist(origin.x()),
               lefdist(origin.y()));
  }

  if (master->getEEQ()) {
    std::string eeq = master->getEEQ()->getName();
    if (_use_master_ids) {
      fmt::print(_out,
                 "    EEQ M{} ;\n",
                 static_cast<std::uint32_t>(master->getEEQ()->getMasterId()));
    } else {
      fmt::print(_out, "    EEQ {} ;\n", eeq.c_str());
    }
  }

  if (master->getLEQ()) {
    std::string leq = master->getLEQ()->getName();
    if (_use_master_ids) {
      fmt::print(_out,
                 "    LEQ M{} ;\n",
                 static_cast<std::uint32_t>(master->getLEQ()->getMasterId()));
    } else {
      fmt::print(_out, "    LEQ {} ;\n", leq.c_str());
    }
  }

  int w = master->getWidth();
  int h = master->getHeight();

  if ((w != 0) || (h != 0)) {
    fmt::print(_out, "    SIZE {:.11g} BY {:.11g} ;\n", lefdist(w), lefdist(h));
  }

  if (master->getSymmetryX() || master->getSymmetryY()
      || master->getSymmetryR90()) {
    fmt::print(_out, "{}", "    SYMMETRY");

    if (master->getSymmetryX()) {
      fmt::print(_out, "{}", " X");
    }

    if (master->getSymmetryY()) {
      fmt::print(_out, "{}", " Y");
    }

    if (master->getSymmetryR90()) {
      fmt::print(_out, "{}", " R90");
    }

    fmt::print(_out, "{}", " ;\n");
  }

  if (origin != Point()) {
    dbTransform t(Point(-origin.x(), -origin.y()));
    master->transform(t);
  }

  if (master->getSite()) {
    std::string site = master->getSite()->getName();
    fmt::print(_out, "    SITE {} ;\n", site.c_str());
  }

  dbSet<dbMTerm> mterms = master->getMTerms();
  dbSet<dbMTerm>::iterator mitr;

  for (mitr = mterms.begin(); mitr != mterms.end(); ++mitr) {
    dbMTerm* mterm = *mitr;
    writeMTerm(mterm);
  }

  dbSet<dbPolygon> poly_obs = master->getPolygonObstructions();
  dbSet<dbBox> obs = master->getObstructions(false);

  if (poly_obs.begin() != poly_obs.end() || obs.begin() != obs.end()) {
    fmt::print(_out, "{}", "    OBS\n");
    writeBoxes(nullptr, poly_obs, "      ");
    writeBoxes(nullptr, obs, "      ");
    fmt::print(_out, "{}", "    END\n");
  }

  if (origin != Point()) {
    dbTransform t(origin);
    master->transform(t);
  }

  if (_use_master_ids) {
    fmt::print(
        _out, "END M{}\n", static_cast<std::uint32_t>(master->getMasterId()));
  } else {
    fmt::print(_out, "END {}\n", name.c_str());
  }
}

void lefout::writeMTerm(dbMTerm* mterm)
{
  std::string name = mterm->getName();

  fmt::print(_out, "    PIN {}\n", name.c_str());
  fmt::print(_out, "        DIRECTION {} ; \n", mterm->getIoType().getString());
  fmt::print(_out, "        USE {} ; \n", mterm->getSigType().getString());

  mterm->writeAntennaLef(*this);
  dbSet<dbMPin> pins = mterm->getMPins();
  dbSet<dbMPin>::iterator pitr;

  for (pitr = pins.begin(); pitr != pins.end(); ++pitr) {
    dbMPin* pin = *pitr;

    dbSet<dbPolygon> poly_geoms = pin->getPolygonGeometry();
    dbSet<dbBox> geoms = pin->getGeometry(false);

    if (poly_geoms.begin() != poly_geoms.end()
        || geoms.begin() != geoms.end()) {
      fmt::print(_out, "        PORT\n");
      writeBoxes(nullptr, poly_geoms, "            ");
      writeBoxes(nullptr, geoms, "            ");
      fmt::print(_out, "        END\n");
    }
  }

  fmt::print(_out, "    END {}\n", name.c_str());
}

void lefout::writePropertyDefinition(dbProperty* prop)
{
  std::string propName = prop->getName();
  dbObjectType owner_type = prop->getPropOwner()->getObjectType();
  dbProperty::Type prop_type = prop->getType();
  std::string objectType, propType;
  switch (owner_type) {
    case dbTechLayerObj:
      objectType = "LAYER";
      break;
    case dbLibObj:
      objectType = "LIBRARY";
      break;
    case dbMasterObj:
      objectType = "MACRO";
      break;
    case dbMPinObj:
      objectType = "PIN";
      break;
    case dbTechViaObj:
      objectType = "VIA";
      break;
    case dbTechViaRuleObj:
      objectType = "VIARULE";
      break;
    case dbTechNonDefaultRuleObj:
      objectType = "NONDEFAULTRULE";
      break;
    default:
      return;
  }

  switch (prop_type) {
    case dbProperty::INT_PROP:
    case dbProperty::BOOL_PROP:
      propType = "INTEGER";
      break;
    case dbProperty::DOUBLE_PROP:
      propType = "REAL";
      break;
    case dbProperty::STRING_PROP:
      propType = "STRING";
      break;
    default:
      return;
  }
  fmt::print(_out,
             "    {} {} {} ",
             objectType.c_str(),
             propName.c_str(),
             propType.c_str());
  if (owner_type == dbLibObj) {
    fmt::print(_out, "{}", "\n        ");
    fmt::print(_out, "{}", dbProperty::writePropValue(prop));
    fmt::print(_out, "{}", "\n    ");
  }

  fmt::print(_out, "{}", ";\n");
}

inline void lefout::writeObjectPropertyDefinitions(
    dbObject* obj,
    std::unordered_map<std::string, int16_t>& propertiesMap)
{
  int bitNumber;
  switch (obj->getObjectType()) {
    case dbTechLayerObj:
      bitNumber = 0;
      break;
    case dbLibObj:
      bitNumber = 1;
      break;
    case dbMasterObj:
      bitNumber = 2;
      break;
    case dbMPinObj:
      bitNumber = 3;
      break;
    case dbTechViaObj:
      bitNumber = 4;
      break;
    case dbTechViaRuleObj:
      bitNumber = 5;
      break;
    case dbTechNonDefaultRuleObj:
      bitNumber = 6;
      break;
    default:
      return;
  }
  dbSet<dbProperty> properties = dbProperty::getProperties(obj);
  dbSet<dbProperty>::iterator pitr;
  for (pitr = properties.begin(); pitr != properties.end(); ++pitr) {
    dbProperty* prop = *pitr;
    if (propertiesMap[prop->getName()] & 0x1 << bitNumber) {
      continue;
    }
    propertiesMap[prop->getName()] |= 0x1 << bitNumber;
    writePropertyDefinition(prop);
  }
}

void lefout::writePropertyDefinitions(dbLib* lib)
{
  std::unordered_map<std::string, int16_t> propertiesMap;
  dbTech* tech = lib->getTech();

  fmt::print(_out, "{}", "\nPROPERTYDEFINITIONS\n");

  // writing property definitions of objectType LAYER
  for (dbTechLayer* layer : tech->getLayers()) {
    writeObjectPropertyDefinitions(layer, propertiesMap);
  }

  // writing property definitions of objectType LIBRARY
  writeObjectPropertyDefinitions(lib, propertiesMap);

  // writing property definitions of objectType MACRO
  for (dbMaster* master : lib->getMasters()) {
    writeObjectPropertyDefinitions(master, propertiesMap);
    for (dbMTerm* term : master->getMTerms()) {
      for (dbMPin* pin : term->getMPins()) {
        writeObjectPropertyDefinitions(pin, propertiesMap);
      }
    }
  }

  // writing property definitions of objectType VIA
  for (dbTechVia* via : tech->getVias()) {
    writeObjectPropertyDefinitions(via, propertiesMap);
  }

  // writing property definitions of objectType VIARULE
  for (dbTechViaRule* vrule : tech->getViaRules()) {
    writeObjectPropertyDefinitions(vrule, propertiesMap);
  }

  // writing property definitions of objectType NONDEFAULTRULE
  for (dbTechNonDefaultRule* nrule : tech->getNonDefaultRules()) {
    writeObjectPropertyDefinitions(nrule, propertiesMap);
  }

  fmt::print(_out, "{}", "END PROPERTYDEFINITIONS\n\n");
}

void lefout::writeTech(dbTech* tech)
{
  _dist_factor = 1.0 / tech->getDbUnitsPerMicron();
  _area_factor = _dist_factor * _dist_factor;
  writeTechBody(tech);

  fmt::print(_out, "END LIBRARY\n");
}

void lefout::writeLib(dbLib* lib)
{
  _dist_factor = 1.0 / lib->getDbUnitsPerMicron();
  _area_factor = _dist_factor * _dist_factor;
  writeHeader(lib);
  writeLibBody(lib);
  fmt::print(_out, "END LIBRARY\n");
}

void lefout::writeTechAndLib(dbLib* lib)
{
  _dist_factor = 1.0 / lib->getDbUnitsPerMicron();
  _area_factor = _dist_factor * _dist_factor;
  dbTech* tech = lib->getTech();
  writeHeader(lib);
  writeTechBody(tech);
  writeLibBody(lib);
  fmt::print(_out, "END LIBRARY\n");
}

void lefout::writeAbstractLef(dbBlock* db_block)
{
  utl::SetAndRestore set_dist(_dist_factor,
                              1.0 / db_block->getDbUnitsPerMicron());
  utl::SetAndRestore set_area(_area_factor, _dist_factor * _dist_factor);

  writeHeader(db_block);
  writeBlock(db_block);
  fmt::print(_out, "END LIBRARY\n");
}

/*** 3D CODE ***/
template <typename GenericBox>
void lefout::write3DBoxes(dbBlock* block,
                        dbSet<GenericBox>& boxes,
                        const char* indent,
                        int tier)
{
  dbTechLayer* cur_layer = nullptr;

  for (GenericBox* generic_box : boxes) {
    if (generic_box == nullptr) {
      continue;
    }

    dbBox* box = generic_box;
    dbTechLayer* layer = box->getTechLayer();

    // Checks if the box is either a tech via or a block via.
    if (box->getTechVia() || box->getBlockVia()) {
      std::string via_name;
      if (box->getTechVia()) {
        via_name = box->getTechVia()->getName();
      }
      if (box->getBlockVia()) {
        via_name = block->getName() + "_" + box->getBlockVia()->getName();
      }

      int x, y;
      box->getViaXY(x, y);
      fmt::print(_out,
                 "{}VIA {:.11g} {:.11g} {} ;\n",
                 indent,
                 lefdist(x),
                 lefdist(y),
                 via_name.c_str());
      cur_layer = nullptr;
    } else {
      std::string layer_name;
      if (_use_alias && layer->hasAlias()) {
        layer_name = layer->getAlias();
      } else {
        layer_name = layer->getName();
      }

      layer_name += "_" + config_3D_pdk_->getTiers()[tier].name;

      if (cur_layer != layer) {
        fmt::print(_out, "{}LAYER {} ;\n", indent, layer_name.c_str());
        cur_layer = layer;
      }

      writeBox(indent, box);
    }
  }
}
template <>
void lefout::write3DBoxes(dbBlock* block,
                        dbSet<dbPolygon>& boxes,
                        const char* indent,
                        int tier)
{
  dbTechLayer* cur_layer = nullptr;

  for (dbPolygon* box : boxes) {
    if (box == nullptr) {
      continue;
    }

    dbTechLayer* layer = box->getTechLayer();

    std::string layer_name;
    if (_use_alias && layer->hasAlias()) {
      layer_name = layer->getAlias();
    } else {
      layer_name = layer->getName();
    }

    layer_name += "_" + config_3D_pdk_->getTiers()[tier].name;

    if (cur_layer != layer) {
      fmt::print(_out, "{}LAYER {} ;\n", indent, layer_name.c_str());
      cur_layer = layer;
    }

    writePolygon(indent, box);
  }
}


void lefout::write3DMTerm(dbMTerm* mterm, int tier)
{
  std::string name = mterm->getName();

  fmt::print(_out, "    PIN {}\n", name.c_str());
  fmt::print(_out, "        DIRECTION {} ; \n", mterm->getIoType().getString());
  fmt::print(_out, "        USE {} ; \n", mterm->getSigType().getString());

  mterm->writeAntennaLef(*this);
  dbSet<dbMPin> pins = mterm->getMPins();
  dbSet<dbMPin>::iterator pitr;

  for (pitr = pins.begin(); pitr != pins.end(); ++pitr) {
    dbMPin* pin = *pitr;

    dbSet<dbPolygon> poly_geoms = pin->getPolygonGeometry();
    dbSet<dbBox> geoms = pin->getGeometry(false);

    if (poly_geoms.begin() != poly_geoms.end()
        || geoms.begin() != geoms.end()) {
      fmt::print(_out, "        PORT\n");
      write3DBoxes(nullptr, poly_geoms, "            ", tier);
      write3DBoxes(nullptr, geoms, "            ", tier);
      fmt::print(_out, "        END\n");
    }
  }

  fmt::print(_out, "    END {}\n", name.c_str());
}

void lefout::write3DObstructions(dbBlock* db_block, int tier)
{
  ObstructionMap obstructions;
  getObstructions(db_block, obstructions);

  fmt::print(_out, "{}", "  OBS\n");
  dbBox* block_bounding_box = db_block->getBBox();
  for (const auto& [tech_layer, polySet] : obstructions) {
    std::string layer_name = tech_layer->getName();
    layer_name += "_" + config_3D_pdk_->getTiers()[tier].name;
    fmt::print(_out, "    LAYER {} ;\n", layer_name.c_str());

    if (bloat_occupied_layers_) {
      writeBox("   ", block_bounding_box);
    } else {
      const int bloat = determineBloat(tech_layer);
      boost::polygon::polygon_90_set_data<int> shrink_poly = polySet;
      shrink_poly.shrink2(bloat, bloat, bloat, bloat);

      // Decompose the polygon set to rectanges in non-preferred direction
      std::vector<boost::polygon::rectangle_data<int>> rects;
      if (tech_layer->getDirection() == odb::dbTechLayerDir::HORIZONTAL) {
        shrink_poly.get_rectangles(rects, boost::polygon::VERTICAL);
      } else if (tech_layer->getDirection() == odb::dbTechLayerDir::VERTICAL) {
        shrink_poly.get_rectangles(rects, boost::polygon::HORIZONTAL);
      } else if (tech_layer->getDirection() == odb::dbTechLayerDir::NONE) {
        shrink_poly.get_rectangles(rects);
      }

      for (const auto& rect : rects) {
        writeRect("   ", rect);
      }
    }
  }
  fmt::print(_out, "  END\n");
}

void lefout::write3DBlockVia(dbBlock* db_block, dbVia* via, int tier)
{
  std::string name = db_block->getName() + "_" + via->getName();

  if (via->isDefault()) {
    fmt::print(_out, "\nVIA {} DEFAULT\n", name.c_str());
  } else {
    fmt::print(_out, "\nVIA {}\n", name.c_str());
  }

  dbTechViaGenerateRule* rule = via->getViaGenerateRule();

  if (rule == nullptr) {
    dbSet<dbBox> boxes = via->getBoxes();
    write3DBoxes(db_block, boxes, "    ", tier);
  } else {
    std::string rname = rule->getName();
    rname += "_" + config_3D_pdk_->getTiers()[tier].name;
    fmt::print(_out, "  VIARULE {} ;\n", rname.c_str());

    const dbViaParams P = via->getViaParams();

    fmt::print(_out,
               "  CUTSIZE {:.11g} {:.11g} ;\n",
               lefdist(P.getXCutSize()),
               lefdist(P.getYCutSize()));

    std::string top = P.getTopLayer()->getName() + config_3D_pdk_->getTiers()[tier].name;
    std::string bot = P.getBottomLayer()->getName() + config_3D_pdk_->getTiers()[tier].name;
    std::string cut = P.getCutLayer()->getName() + config_3D_pdk_->getTiers()[tier].name;
    fmt::print( _out, "  LAYERS {} {} {} ;\n", bot.c_str(), cut.c_str(), top.c_str());
    
    fmt::print(_out,
               "  CUTSPACING {:.11g} {:.11g} ;\n",
               lefdist(P.getXCutSpacing()),
               lefdist(P.getYCutSpacing()));
    fmt::print(_out,
               "  ENCLOSURE {:.11g} {:.11g} {:.11g} {:.11g} ;\n",
               lefdist(P.getXBottomEnclosure()),
               lefdist(P.getYBottomEnclosure()),
               lefdist(P.getXTopEnclosure()),
               lefdist(P.getYTopEnclosure()));

    if ((P.getNumCutRows() != 1) || (P.getNumCutCols() != 1)) {
      fmt::print(
          _out, "  ROWCOL {} {} ;\n", P.getNumCutRows(), P.getNumCutCols());
    }

    if ((P.getXOrigin() != 0) || (P.getYOrigin() != 0)) {
      fmt::print(_out,
                 "  ORIGIN {:.11g} {:.11g} ;\n",
                 lefdist(P.getXOrigin()),
                 lefdist(P.getYOrigin()));
    }

    if ((P.getXTopOffset() != 0) || (P.getYTopOffset() != 0)
        || (P.getXBottomOffset() != 0) || (P.getYBottomOffset() != 0)) {
      fmt::print(_out,
                 "  OFFSET {:.11g} {:.11g} {:.11g} {:.11g} ;\n",
                 lefdist(P.getXBottomOffset()),
                 lefdist(P.getYBottomOffset()),
                 lefdist(P.getXTopOffset()),
                 lefdist(P.getYTopOffset()));
    }

    std::string pname = via->getPattern();
    if (strcmp(pname.c_str(), "") != 0) {
      fmt::print(_out, "  PATTERNNAME {} ;\n", pname.c_str());
    }
  }

  fmt::print(_out, "END {}\n", name.c_str());
}

void lefout::write3DBlock(dbBlock* db_block, int tier)
{
  dbBox* bounding_box = db_block->getBBox();
  double size_x = lefdist(bounding_box->xMax());
  double size_y = lefdist(bounding_box->yMax());

  for (auto via : db_block->getVias()) {
    write3DBlockVia(db_block, via, tier);
  }
  std::string block = db_block->getName();
  block += "_" + config_3D_pdk_->getTiers()[tier].name;

  fmt::print(_out, "\nMACRO {}\n", block.c_str());
  fmt::print(_out, "  FOREIGN {} 0 0 ;\n", block.c_str());
  fmt::print(_out, "  CLASS BLOCK ;\n");
  fmt::print(_out, "  SIZE {:.11g} BY {:.11g} ;\n", size_x, size_y);
  write3DPins(db_block, tier);
  write3DObstructions(db_block, tier);
  fmt::print(_out, "END {}\n", block.c_str());
}

void lefout::write3DPins(dbBlock* db_block, int tier)
{
  write3DPowerPins(db_block, tier);
  write3DBlockTerms(db_block, tier);
}

void lefout::write3DBlockTerms(dbBlock* db_block, int tier)
{
  for (dbBTerm* b_term : db_block->getBTerms()) {
    fmt::print(_out, "  PIN {}\n", b_term->getName().c_str());
    fmt::print(_out, "    DIRECTION {} ;\n", b_term->getIoType().getString());

    std::string sig_type = "SIGNAL";
    switch (b_term->getSigType().getValue()) {
      case odb::dbSigType::SIGNAL:
      case odb::dbSigType::SCAN:
      case odb::dbSigType::TIEOFF:
      case odb::dbSigType::RESET:
        // LEF Pins can only be pin : [USE { SIGNAL | ANALOG | POWER | GROUND |
        // CLOCK } ;] whereas nets can be net: [+ USE {ANALOG | CLOCK | GROUND |
        // POWER | RESET | SCAN | SIGNAL | TIEOFF}] BTerms get their sigType
        // from the net. So we map other sigtypes to SIGNAL in exported LEFs
        sig_type = "SIGNAL";
        break;
      case odb::dbSigType::ANALOG:
        sig_type = "ANALOG";
        break;
      case odb::dbSigType::POWER:
        sig_type = "POWER";
        break;
      case odb::dbSigType::GROUND:
        sig_type = "GROUND";
        break;
      case odb::dbSigType::CLOCK:
        sig_type = "CLOCK";
        break;
    }
    fmt::print(_out, "    USE {} ;\n", sig_type);

    for (dbBPin* db_b_pin : b_term->getBPins()) {
      fmt::print(_out, "{}", "    PORT\n");
      dbSet<dbBox> term_pins = db_b_pin->getBoxes();
      write3DBoxes(db_block, term_pins, "      ", tier);
      fmt::print(_out, "{}", "    END\n");
    }
    fmt::print(_out, "  END {}\n", b_term->getName().c_str());
  }
}

void lefout::write3DPowerPins(dbBlock* db_block, int tier)
{  // Power Ground.
  for (dbNet* net : db_block->getNets()) {
    if (!net->getSigType().isSupply()) {
      continue;
    }
    if (net->get1stBTerm() != nullptr) {
      // net already has pins that will be added
      continue;
    }
    fmt::print(_out, "  PIN {}\n", net->getName().c_str());
    fmt::print(_out, "    USE {} ;\n", net->getSigType().getString());
    fmt::print(
        _out, "    DIRECTION {} ;\n", dbIoType(dbIoType::INOUT).getString());
    for (dbSWire* special_wire : net->getSWires()) {
      fmt::print(_out, "    PORT\n");
      dbSet<dbSBox> wires = special_wire->getWires();
      write3DBoxes(db_block, wires, /*indent=*/"      ", tier);
      fmt::print(_out, "    END\n");
    }
    fmt::print(_out, "  END {}\n", net->getName().c_str());
  }
}

void lefout::write3DNonDefaultRule(dbTech* tech, dbTechNonDefaultRule* rule, int tier)
{
  std::string name = rule->getName();
  fmt::print(_out, "\nNONDEFAULTRULE {}\n", name.c_str());

  if (rule->getHardSpacing()) {
    fmt::print(_out, "{}", "HARDSPACING ;\n");
  }

  std::vector<dbTechLayerRule*> layer_rules;
  rule->getLayerRules(layer_rules);

  std::vector<dbTechLayerRule*>::iterator litr;
  for (litr = layer_rules.begin(); litr != layer_rules.end(); ++litr) {
    write3DLayerRule(*litr, tier);
  }

  std::vector<dbTechVia*> vias;
  rule->getVias(vias);

  std::vector<dbTechVia*>::iterator vitr;
  for (vitr = vias.begin(); vitr != vias.end(); ++vitr) {
    write3DVia(*vitr, tier);
  }

  std::vector<dbTechSameNetRule*> srules;
  rule->getSameNetRules(srules);

  if (srules.begin() != srules.end()) {
    fmt::print(_out, "\nSPACING\n");

    std::vector<dbTechSameNetRule*>::iterator sritr;
    for (sritr = srules.begin(); sritr != srules.end(); ++sritr) {
      write3DSameNetRule(*sritr, tier);
    }

    fmt::print(_out, "\nEND SPACING\n");
  }

  std::vector<dbTechVia*> use_vias;
  rule->getUseVias(use_vias);

  std::vector<dbTechVia*>::iterator uvitr;
  for (uvitr = use_vias.begin(); uvitr != use_vias.end(); ++uvitr) {
    dbTechVia* via = *uvitr;
    std::string vname = via->getName();
    vname += "_" + config_3D_pdk_->getTiers()[tier].name;
    fmt::print(_out, "USEVIA {} ;\n", vname.c_str());
  }

  std::vector<dbTechViaGenerateRule*> use_rules;
  rule->getUseViaRules(use_rules);

  std::vector<dbTechViaGenerateRule*>::iterator uvritr;
  for (uvritr = use_rules.begin(); uvritr != use_rules.end(); ++uvritr) {
    dbTechViaGenerateRule* rule = *uvritr;
    std::string rname = rule->getName();
    rname += "_" + config_3D_pdk_->getTiers()[tier].name;
    fmt::print(_out, "USEVIARULE {} ;\n", rname.c_str());
  }

  dbSet<dbTechLayer> layers = tech->getLayers();
  dbSet<dbTechLayer>::iterator layitr;

  for (layitr = layers.begin(); layitr != layers.end(); ++layitr) {
    dbTechLayer* layer = *layitr;
    int count;

    if (rule->getMinCuts(layer, count)) {
      std::string lname = layer->getName();
      lname += "_" + config_3D_pdk_->getTiers()[tier].name;
      fmt::print(_out, "MINCUTS {} {} ;\n", lname.c_str(), count);
    }
  }

  fmt::print(_out, "\nEND {}\n", name.c_str());
}

void lefout::write3DLayerRule(dbTechLayerRule* rule, int tier)
{
  dbTechLayer* layer = rule->getLayer();
  std::string name;
  if (_use_alias && layer->hasAlias()) {
    name = layer->getAlias();
  } else {
    name = layer->getName();
  }
  name += "_" + config_3D_pdk_->getTiers()[tier].name;
  fmt::print(_out, "\nLAYER {}\n", name.c_str());

  if (rule->getWidth()) {
    fmt::print(_out, "    WIDTH {:.11g} ;\n", lefdist(rule->getWidth()));
  }

  if (rule->getSpacing()) {
    fmt::print(_out, "    SPACING {:.11g} ;\n", lefdist(rule->getSpacing()));
  }

  if (rule->getWireExtension() != 0.0) {
    fmt::print(_out,
               "    WIREEXTENSION {:.11g} ;\n",
               lefdist(rule->getWireExtension()));
  }

  if (rule->getResistance() != 0.0) {
    fmt::print(
        _out, "    RESISTANCE RPERSQ {:.11g} ;\n", rule->getResistance());
  }

  if (rule->getCapacitance() != 0.0) {
    fmt::print(
        _out, "    CAPACITANCE CPERSQDIST {:.11g} ;\n", rule->getCapacitance());
  }

  if (rule->getEdgeCapacitance() != 0.0) {
    fmt::print(
        _out, "      EDGECAPACITANCE {:.11g} ;\n", rule->getEdgeCapacitance());
  }

  fmt::print(_out, "END {}\n", name.c_str());
}

void lefout::write3DTechViaRule(dbTechViaRule* rule, int tier)
{
  std::string name = rule->getName();
  fmt::print(_out, "\nVIARULE {}\n", name.c_str());

  uint idx;

  for (idx = 0; idx < rule->getViaLayerRuleCount(); ++idx) {
    dbTechViaLayerRule* layrule = rule->getViaLayerRule(idx);
    dbTechLayer* layer = layrule->getLayer();
    std::string lname = layer->getName();
    lname += "_" + config_3D_pdk_->getTiers()[tier].name;
    fmt::print(_out, "    LAYER {} ;\n", lname.c_str());

    if (layrule->getDirection() == dbTechLayerDir::VERTICAL) {
      fmt::print(_out, "      DIRECTION VERTICAL ;\n");
    } else if (layrule->getDirection() == dbTechLayerDir::HORIZONTAL) {
      fmt::print(_out, "      DIRECTION HORIZONTAL ;\n");
    }

    if (layrule->hasWidth()) {
      int minW, maxW;
      layrule->getWidth(minW, maxW);
      fmt::print(_out,
                 "      WIDTH {:.11g} TO {:.11g} ;\n",
                 lefdist(minW),
                 lefdist(maxW));
    }
  }

  for (idx = 0; idx < rule->getViaCount(); ++idx) {
    dbTechVia* via = rule->getVia(idx);
    std::string vname = via->getName();
    vname += "_" + config_3D_pdk_->getTiers()[tier].name;
    fmt::print(_out, "    VIA {} ;\n", vname.c_str());
  }

  fmt::print(_out, "END {}\n", name.c_str());
}

void lefout::write3DTechViaGenerateRule(dbTechViaGenerateRule* rule, int tier)
{
  std::string name = rule->getName();
  name += "_" + config_3D_pdk_->getTiers()[tier].name;

  if (rule->isDefault()) {
    fmt::print(_out, "\nVIARULE {} GENERATE DEFAULT\n", name.c_str());
  } else {
    fmt::print(_out, "\nVIARULE {} GENERATE \n", name.c_str());
  }

  uint idx;

  for (idx = 0; idx < rule->getViaLayerRuleCount(); ++idx) {
    dbTechViaLayerRule* layrule = rule->getViaLayerRule(idx);
    dbTechLayer* layer = layrule->getLayer();
    std::string lname = layer->getName();
    lname += "_" + config_3D_pdk_->getTiers()[tier].name;
    fmt::print(_out, "    LAYER {} ;\n", lname.c_str());

    if (layrule->getDirection() == dbTechLayerDir::VERTICAL) {
      fmt::print(_out, "      DIRECTION VERTICAL ;\n");
    } else if (layrule->getDirection() == dbTechLayerDir::HORIZONTAL) {
      fmt::print(_out, "      DIRECTION HORIZONTAL ;\n");
    }

    if (layrule->hasOverhang()) {
      fmt::print(
          _out, "      OVERHANG {:.11g} ;\n", lefdist(layrule->getOverhang()));
    }

    if (layrule->hasMetalOverhang()) {
      fmt::print(_out,
                 "      METALOVERHANG {:.11g} ;\n",
                 lefdist(layrule->getMetalOverhang()));
    }

    if (layrule->hasEnclosure()) {
      int overhang1, overhang2;
      layrule->getEnclosure(overhang1, overhang2);
      fmt::print(_out,
                 "      ENCLOSURE {:.11g} {:.11g} ;\n",
                 lefdist(overhang1),
                 lefdist(overhang2));
    }

    if (layrule->hasWidth()) {
      int minW, maxW;
      layrule->getWidth(minW, maxW);
      fmt::print(_out,
                 "      WIDTH {:.11g} TO {:.11g} ;\n",
                 lefdist(minW),
                 lefdist(maxW));
    }

    if (layrule->hasRect()) {
      Rect r;
      layrule->getRect(r);
      fmt::print(_out,
                 "      RECT  {:.11g} {:.11g}  {:.11g} {:.11g}  ;\n",
                 lefdist(r.xMin()),
                 lefdist(r.yMin()),
                 lefdist(r.xMax()),
                 lefdist(r.yMax()));
    }

    if (layrule->hasSpacing()) {
      int spacing_x, spacing_y;
      layrule->getSpacing(spacing_x, spacing_y);
      fmt::print(_out,
                 "      SPACING {:.11g} BY {:.11g} ;\n",
                 lefdist(spacing_x),
                 lefdist(spacing_y));
    }

    if (layrule->hasResistance()) {
      fmt::print(
          _out, "      RESISTANCE {:.11g} ;\n", layrule->getResistance());
    }
  }

  fmt::print(_out, "END {}\n", name.c_str());
}

void lefout::write3DSameNetRule(dbTechSameNetRule* rule, int tier)
{
  dbTechLayer* l1 = rule->getLayer1();
  dbTechLayer* l2 = rule->getLayer2();

  std::string n1;
  if (_use_alias && l1->hasAlias()) {
    n1 = l1->getAlias();
  } else {
    n1 = l1->getName();
  }

  std::string n2;
  if (_use_alias && l2->hasAlias()) {
    n2 = l2->getAlias();
  } else {
    n2 = l2->getName();
  }

  if (rule->getAllowStackedVias()) {
    fmt::print(_out,
               "  SAMENET {} {} {:.11g} STACK ;\n",
               n1.c_str(),
               n2.c_str(),
               lefdist(rule->getSpacing()));
  } else {
    fmt::print(_out,
               "  SAMENET {} {} {:.11g} ;\n",
               n1.c_str(),
               n2.c_str(),
               lefdist(rule->getSpacing()));
  }
}

void lefout::write3DVia(dbTechVia* via, int tier)
{
  std::string name = via->getName();
  name += "_" + config_3D_pdk_->getTiers()[tier].name;

  if (via->isDefault()) {
    fmt::print(_out, "\nVIA {} DEFAULT\n", name.c_str());
  } else {
    fmt::print(_out, "\nVIA {}\n", name.c_str());
  }

  if (via->isTopOfStack()) {
    fmt::print(_out, "    TOPOFSTACKONLY\n");
  }

  if (via->getResistance() != 0.0) {
    fmt::print(_out, "    RESISTANCE {:.11g} ;\n", via->getResistance());
  }

  dbTechViaGenerateRule* rule = via->getViaGenerateRule();

  if (rule == nullptr) {
    dbSet<dbBox> boxes = via->getBoxes();
    write3DBoxes(nullptr, boxes, "    ", tier);
  } else {
    std::string rname = rule->getName();
    rname += "_" + config_3D_pdk_->getTiers()[tier].name;
    fmt::print(_out, "\n    VIARULE {} \n", rname.c_str());

    const dbViaParams P = via->getViaParams();

    fmt::print(_out,
               " + CUTSIZE {:.11g} {:.11g}\n",
               lefdist(P.getXCutSize()),
               lefdist(P.getYCutSize()));
    std::string top = P.getTopLayer()->getName();
    std::string bot = P.getBottomLayer()->getName();
    std::string cut = P.getCutLayer()->getName();
    fmt::print(
        _out, " + LAYERS {} {} {}\n", bot.c_str(), cut.c_str(), top.c_str());
    fmt::print(_out,
               " + CUTSPACING {:.11g} {:.11g}\n",
               lefdist(P.getXCutSpacing()),
               lefdist(P.getYCutSpacing()));
    fmt::print(_out,
               " + ENCLOSURE {:.11g} {:.11g} {:.11g} {:.11g}\n",
               lefdist(P.getXBottomEnclosure()),
               lefdist(P.getYBottomEnclosure()),
               lefdist(P.getXTopEnclosure()),
               lefdist(P.getYTopEnclosure()));

    if ((P.getNumCutRows() != 1) || (P.getNumCutCols() != 1)) {
      fmt::print(
          _out, " + ROWCOL {} {}\n", P.getNumCutRows(), P.getNumCutCols());
    }

    if ((P.getXOrigin() != 0) || (P.getYOrigin() != 0)) {
      fmt::print(_out,
                 " + ORIGIN {:.11g} {:.11g}\n",
                 lefdist(P.getXOrigin()),
                 lefdist(P.getYOrigin()));
    }

    if ((P.getXTopOffset() != 0) || (P.getYTopOffset() != 0)
        || (P.getXBottomOffset() != 0) || (P.getYBottomOffset() != 0)) {
      fmt::print(_out,
                 " + OFFSET {:.11g} {:.11g} {:.11g} {:.11g}\n",
                 lefdist(P.getXBottomOffset()),
                 lefdist(P.getYBottomOffset()),
                 lefdist(P.getXTopOffset()),
                 lefdist(P.getYTopOffset()));
    }

    std::string pname = via->getPattern();
    if (strcmp(pname.c_str(), "") != 0) {
      fmt::print(_out, " + PATTERNNAME {}\n", pname.c_str());
    }
  }

  fmt::print(_out, "END {}\n", name.c_str());
}

void lefout::write3DLayer(dbTechLayer* layer, int tier)
{
  std::string name;
  if (_use_alias && layer->hasAlias()) {
    name = layer->getAlias();
  } else {
    name = layer->getName();
  }
  name += "_" + config_3D_pdk_->getTiers()[tier].name;

  fmt::print(_out, "\nLAYER {}\n", name.c_str());
  fmt::print(_out, "    TYPE {} ;\n", layer->getType().getString());

  if (layer->getNumMasks() > 1) {
    fmt::print(_out, "    MASK {} ;\n", layer->getNumMasks());
  }

  if (layer->getPitch()) {
    fmt::print(_out, "    PITCH {:.11g} ;\n", lefdist(layer->getPitch()));
  }

  if (layer->getWidth()) {
    fmt::print(_out, "    WIDTH {:.11g} ;\n", lefdist(layer->getWidth()));
  }

  if (layer->getWireExtension() != 0.0) {
    fmt::print(_out,
               "    WIREEXTENSION {:.11g} ;\n",
               lefdist(layer->getWireExtension()));
  }

  if (layer->hasArea()) {
    fmt::print(_out, "    AREA {:.11g} ;\n", layer->getArea());
  }

  uint thickness;
  if (layer->getThickness(thickness)) {
    fmt::print(_out, "    THICKNESS {:.11g} ;\n", lefdist(thickness));
  }

  if (layer->hasMaxWidth()) {
    fmt::print(_out, "    MAXWIDTH {:.11g} ;\n", lefdist(layer->getMaxWidth()));
  }

  if (layer->hasMinStep()) {
    fmt::print(_out, "    MINSTEP {:.11g} ;\n", lefdist(layer->getMinStep()));
  }

  if (layer->hasProtrusion()) {
    fmt::print(_out,
               "    PROTRUSIONWIDTH {:.11g}  LENGTH {:.11g}  WIDTH {:.11g} ;\n",
               lefdist(layer->getProtrusionWidth()),
               lefdist(layer->getProtrusionLength()),
               lefdist(layer->getProtrusionFromWidth()));
  }

  for (auto rule : layer->getV54SpacingRules()) {
    rule->writeLef(*this);
  }

  if (layer->hasV55SpacingRules()) {
    layer->printV55SpacingRules(*this);
    auto inf_rules = layer->getV55InfluenceRules();
    if (!inf_rules.empty()) {
      fmt::print(_out, "SPACINGTABLE INFLUENCE");
      for (auto rule : inf_rules) {
        rule->writeLef(*this);
      }
      fmt::print(_out, " ;\n");
    }
  }

  std::vector<dbTechMinCutRule*> cut_rules;
  std::vector<dbTechMinCutRule*>::const_iterator citr;
  if (layer->getMinimumCutRules(cut_rules)) {
    for (citr = cut_rules.begin(); citr != cut_rules.end(); citr++) {
      (*citr)->writeLef(*this);
    }
  }

  std::vector<dbTechMinEncRule*> enc_rules;
  std::vector<dbTechMinEncRule*>::const_iterator eitr;
  if (layer->getMinEnclosureRules(enc_rules)) {
    for (eitr = enc_rules.begin(); eitr != enc_rules.end(); eitr++) {
      (*eitr)->writeLef(*this);
    }
  }

  layer->writeAntennaRulesLef(*this);

  if (layer->getDirection() != dbTechLayerDir::NONE) {
    fmt::print(_out, "    DIRECTION {} ;\n", layer->getDirection().getString());
  }

  if (layer->getResistance() != 0.0) {
    if (layer->getType() == dbTechLayerType::CUT) {
      fmt::print(_out, "    RESISTANCE {:.11g} ;\n", layer->getResistance());
    } else {
      fmt::print(
          _out, "    RESISTANCE RPERSQ {:.11g} ;\n", layer->getResistance());
    }
  }

  if (layer->getCapacitance() != 0.0) {
    fmt::print(_out,
               "    CAPACITANCE CPERSQDIST {:.11g} ;\n",
               layer->getCapacitance());
  }

  if (layer->getEdgeCapacitance() != 0.0) {
    fmt::print(
        _out, "    EDGECAPACITANCE {:.11g} ;\n", layer->getEdgeCapacitance());
  }

  fmt::print(_out, "{}", dbProperty::writeProperties(layer));

  fmt::print(_out, "END {}\n", name.c_str());
}

void lefout::write3DMaster(dbMaster* master, int tier)
{
  std::string name = master->getName();

  if (_use_master_ids) {
    fmt::print(_out, "\nMACRO M{}_{}\n", static_cast<std::uint32_t>(master->getMasterId()), config_3D_pdk_->getTiers()[tier].name);
  } else {
    fmt::print(_out, "\nMACRO {}_{}\n", name.c_str(), config_3D_pdk_->getTiers()[tier].name);
  }

  fmt::print(_out, "    CLASS {} ;\n", master->getType().getString());

  const odb::Point origin = master->getOrigin();

  if (origin != Point()) {
    fmt::print(_out,
               "    ORIGIN {:.11g} {:.11g} ;\n",
               lefdist(origin.x()),
               lefdist(origin.y()));
  }

  if (master->getEEQ()) {
    std::string eeq = master->getEEQ()->getName();
    if (_use_master_ids) {
      fmt::print(_out,
                 "    EEQ M{} ;\n",
                 static_cast<std::uint32_t>(master->getEEQ()->getMasterId()));
    } else {
      fmt::print(_out, "    EEQ {} ;\n", eeq.c_str());
    }
  }

  if (master->getLEQ()) {
    std::string leq = master->getLEQ()->getName();
    if (_use_master_ids) {
      fmt::print(_out,
                 "    LEQ M{} ;\n",
                 static_cast<std::uint32_t>(master->getLEQ()->getMasterId()));
    } else {
      fmt::print(_out, "    LEQ {} ;\n", leq.c_str());
    }
  }

  int w = master->getWidth();
  int h = master->getHeight();

  if ((w != 0) || (h != 0)) {
    fmt::print(_out, "    SIZE {:.11g} BY {:.11g} ;\n", lefdist(w), lefdist(h));
  }

  if (master->getSymmetryX() || master->getSymmetryY()
      || master->getSymmetryR90()) {
    fmt::print(_out, "{}", "    SYMMETRY");

    if (master->getSymmetryX()) {
      fmt::print(_out, "{}", " X");
    }

    if (master->getSymmetryY()) {
      fmt::print(_out, "{}", " Y");
    }

    if (master->getSymmetryR90()) {
      fmt::print(_out, "{}", " R90");
    }

    fmt::print(_out, "{}", " ;\n");
  }

  if (origin != Point()) {
    dbTransform t(Point(-origin.x(), -origin.y()));
    master->transform(t);
  }

  if (master->getSite()) {
    std::string site = master->getSite()->getName();
    fmt::print(_out, "    SITE {} ;\n", site.c_str());
  }

  dbSet<dbMTerm> mterms = master->getMTerms();
  dbSet<dbMTerm>::iterator mitr;

  for (mitr = mterms.begin(); mitr != mterms.end(); ++mitr) {
    dbMTerm* mterm = *mitr;
    write3DMTerm(mterm, tier);
  }

  dbSet<dbPolygon> poly_obs = master->getPolygonObstructions();
  dbSet<dbBox> obs = master->getObstructions(false);

  if (poly_obs.begin() != poly_obs.end() || obs.begin() != obs.end()) {
    fmt::print(_out, "{}", "    OBS\n");
    write3DBoxes(nullptr, poly_obs, "      ", tier);
    write3DBoxes(nullptr, obs, "      ", tier);
    fmt::print(_out, "{}", "    END\n");
  }

  if (origin != Point()) {
    dbTransform t(origin);
    master->transform(t);
  }

  if (_use_master_ids) {
    fmt::print(_out, "END M{}_{}\n", static_cast<std::uint32_t>(master->getMasterId()), config_3D_pdk_->getTiers()[tier].name);
  } else {
    fmt::print(_out, "END {}_{}\n", name.c_str(), config_3D_pdk_->getTiers()[tier].name);
  }
}

void lefout::write3DTechBody(const std::vector<dbTech*>& techs, odb::dbSet<odb::dbTech> MIV_tech)
{
  // Determine integration with next tier
  std::string integration_type = "F2F";  // default
  std::string previous_integration_type = "F2F";  // default
  std::vector<dbTechLayer*> miv_layers;

  if (techs.empty()) return;

  const auto& tiers = config_3D_pdk_->getTiers();
  const auto& integration_stack = config_3D_pdk_->getIntegrationStack();

  if (tiers.empty()) return;

  // --- Write general tech info for the first tech (arbitrary) ---
  writeGeneralTechInfo(techs[0]);

  for (size_t tier_idx = 0; tier_idx < tiers.size(); ++tier_idx) {
      const auto& tier = tiers[tier_idx];
      dbTech* tier_tech = findTechForTier(techs, tier);

      std::vector<dbTechLayer*> tier_layers;
      collectTierLayers(tier_tech, tier, tier_layers);

      if (tier_idx < integration_stack.size()) {
        const auto& link = integration_stack[tier_idx];
        integration_type = link.integration_type;

        // Check if previous integration type matches expected relations
        if (!previous_integration_type.empty()) {
            if (!isValidIntegrationTransition(previous_integration_type, integration_type)) {
                logger_->error(utl::ODB, 902,
                    "Integration type mismatch between tier {} and tier {}: previous = {}, current = {}",
                    tiers[tier_idx - 1].name, tier.name,
                    previous_integration_type, integration_type);
            }
        }

        // Synthesize MIV for this tier
        miv_layers = synthesizeMIVLayer(tier_idx, MIV_tech);

        // Update previous integration type
        previous_integration_type = integration_type;

        // Write current tier layers
        writeTierLayers(tier_tech, tier, tier_layers, tier_idx, integration_type);

        // Write MIV if exists connecting to next tier
        for (auto* miv_layer : miv_layers) {
            writeLayer(miv_layer);
        }
      }
      else { // Write last Tier
          // Write current tier layers
          if (previous_integration_type == "F2F")
            writeTierLayers(tier_tech, tier, tier_layers, tier_idx, "B2B");
          else if (previous_integration_type == "F2B")
            writeTierLayers(tier_tech, tier, tier_layers, tier_idx, "F2F");
          else if (previous_integration_type == "B2B")
            writeTierLayers(tier_tech, tier, tier_layers, tier_idx, "F2F");
      }
  }

  // For each tier, collect the last layer before MIV, the MIV layer, and the first layer after MIV
  std::vector<dbTechLayer*> miv_table;
  std::vector<size_t> miv_indexes;
  for (size_t tier_idx = 0; tier_idx < tiers.size(); ++tier_idx) {
    dbTech* tier_tech = findTechForTier(techs, tiers[tier_idx]);
    std::vector<dbTechLayer*> tier_layers;
    collectTierLayers(tier_tech, tiers[tier_idx], tier_layers);

    dbTechLayer* last_before_miv = nullptr;
    dbTechLayer* first_after_miv = nullptr;

    if (!tier_layers.empty()) {
      last_before_miv = tier_layers.back();
    }

    dbTechLayer* miv_layer = nullptr;
    std::vector<dbTechLayer*> mivs_layer;
    int index;
    if (tier_idx < integration_stack.size()) {
      const auto& link = integration_stack[tier_idx];
      index = 0;
      for (const auto& ic : link.interconnects) {
        if (ic.type == "CUT") {
          miv_layer = dbTechLayer::create(tier_tech, ic.name.c_str(), dbTechLayerType::CUT);
          mivs_layer.push_back(miv_layer);
          miv_indexes.push_back(index);
          index++;
        }
      }
    }

    // For the next tier, get the first layer after MIV
    if (tier_idx + 1 < tiers.size()) {
      dbTech* next_tier_tech = findTechForTier(techs, tiers[tier_idx + 1]);
      std::vector<dbTechLayer*> next_tier_layers;
      collectTierLayers(next_tier_tech, tiers[tier_idx + 1], next_tier_layers);
      if (!next_tier_layers.empty()) {
        first_after_miv = next_tier_layers.front();
      }
    }

    // Add to table if all three layers exist
    for (auto* miv_layer : mivs_layer) {
      if (last_before_miv && miv_layer && first_after_miv) {
        miv_table.push_back(last_before_miv);
        miv_table.push_back(miv_layer);
        miv_table.push_back(first_after_miv);
      }
    }
  }

  // --- Write VIAs, spacing rules, and non-default rules for all techs ---
  for (size_t idx = 0; idx < techs.size(); ++idx) {
      dbTech* tech = techs[idx];
      // miv_layer is available from previous loop, or can be recomputed if needed
      writeViasAndRules(idx, tech, miv_table, miv_indexes);
  }
}

void lefout::writeTierWithIntegration(size_t tier_idx,
                                      const std::vector<dbTechLayer*>& tier_layers,
                                      dbTechLayer* miv_layer,
                                      const std::string& integration_type)
{
    if (integration_type == "F2F") {
        for (auto* l : tier_layers) write3DLayer(l, tier_idx);          // bottom
        if (miv_layer) write3DLayer(miv_layer, tier_idx);               // MIV
        for (auto it = tier_layers.rbegin(); it != tier_layers.rend(); ++it)
            write3DLayer(*it, tier_idx);                                // top
    } else if (integration_type == "F2B") {
        for (auto* l : tier_layers) write3DLayer(l, tier_idx);          // bottom
        if (miv_layer) write3DLayer(miv_layer, tier_idx);               // MIV
        for (auto* l : tier_layers) write3DLayer(l, tier_idx);          // top
    } else if (integration_type == "B2B") {
        for (auto it = tier_layers.rbegin(); it != tier_layers.rend(); ++it)
            write3DLayer(*it, tier_idx);                                // bottom
        if (miv_layer) write3DLayer(miv_layer, tier_idx);               // MIV
        for (auto* l : tier_layers) write3DLayer(l, tier_idx);          // top
    } else {
        // default F2F
        for (auto* l : tier_layers) write3DLayer(l, tier_idx);
        if (miv_layer) write3DLayer(miv_layer, tier_idx);
        for (auto it = tier_layers.rbegin(); it != tier_layers.rend(); ++it)
            write3DLayer(*it, tier_idx);
    }
}

void lefout::writeViasAndRules(int tier_idx, dbTech* tech, std::vector<dbTechLayer*> miv_table, std::vector<size_t> miv_indexes)
{ 
  // Check tier index bounds
  const auto& tiers = config_3D_pdk_->getTiers();
  if (tier_idx >= tiers.size()) {
      logger_->warn(utl::ODB, 910, "Tier index {} out of range for VIAs/rules", tier_idx);
      return;
  }

  // --- Write existing VIAs ---
  dbSet<dbTechVia> vias = tech->getVias();
  for (auto* via : vias) {
      write3DVia(via, tier_idx);
  }

  // --- Write existing VIARULEs ---
  dbSet<dbTechViaRule> via_rules = tech->getViaRules();
  for (auto* rule : via_rules) {
      write3DTechViaRule(rule, tier_idx);
  }

  dbSet<dbTechViaGenerateRule> via_gen_rules = tech->getViaGenerateRules();
  for (auto* rule : via_gen_rules) {
      write3DTechViaGenerateRule(rule, tier_idx);
  }

  // --- Synthesize VIA for MIV if it exists ---
  if (!miv_table.empty()) {
    // For each MIV triple: [last_before_miv, miv_layer, first_after_miv]
    for (size_t i = 0; i + 2 < miv_table.size(); i += 3) {
      dbTechLayer* bottom_layer = miv_table[i];
      dbTechLayer* miv_layer = miv_table[i + 1];
      dbTechLayer* top_layer = miv_table[i + 2];

      std::string via_name = miv_layer->getName();
      via_name += "_VIA";

      dbTechVia* miv_via = dbTechVia::create(tech, via_name.c_str());

      // Set properties for the MIV VIA (example: use MIV layer's width/thickness)
      if (miv_via) {
        dbViaParams miv_via_params = dbViaParams();
        miv_via_params.setBottomLayer(bottom_layer);
        miv_via_params.setCutLayer(miv_layer);
        miv_via_params.setTopLayer(top_layer);
        miv_via_params.setXBottomEnclosure(config_3D_pdk_->getIntegrationStack()[tier_idx].interconnects[miv_indexes[i / 3]].bottom_enclosure.first);
        miv_via_params.setYBottomEnclosure(config_3D_pdk_->getIntegrationStack()[tier_idx].interconnects[miv_indexes[i / 3]].bottom_enclosure.second);
        miv_via_params.setXTopEnclosure(config_3D_pdk_->getIntegrationStack()[tier_idx].interconnects[miv_indexes[i / 3]].top_enclosure.first);
        miv_via_params.setYTopEnclosure(config_3D_pdk_->getIntegrationStack()[tier_idx].interconnects[miv_indexes[i / 3]].top_enclosure.second);
        miv_via->setViaParams(miv_via_params);
        miv_via->setResistance(miv_layer->getResistance());

        writeVia(miv_via);
      }
    }
  }

  // --- SameNet rules ---
  std::vector<dbTechSameNetRule*> srules;
  tech->getSameNetRules(srules);
  if (!srules.empty()) {
      fmt::print(_out, "\nSPACING\n");
      for (auto* r : srules) write3DSameNetRule(r, tier_idx);
      fmt::print(_out, "\nEND SPACING\n");
  }

  // --- NonDefault rules ---
  dbSet<dbTechNonDefaultRule> rules = tech->getNonDefaultRules();
  for (auto* r : rules) write3DNonDefaultRule(tech, r, tier_idx);
}

void lefout::write3DLibBody(dbLib* lib, int tier_index)
{
  dbSet<dbSite> sites = lib->getSites();
  dbSet<dbSite>::iterator site_itr;

  for (site_itr = sites.begin(); site_itr != sites.end(); ++site_itr) {
    dbSite* site = *site_itr;
    writeSite(site);
  }

  dbSet<dbMaster> masters = lib->getMasters();
  dbSet<dbMaster>::iterator master_itr;

  for (master_itr = masters.begin(); master_itr != masters.end();
       ++master_itr) {
    dbMaster* master = *master_itr;
    if (_write_marked_masters && !master->isMarked()) {
      continue;
    }
    write3DMaster(master, tier_index);
  }
}

void lefout::write3DTech(const std::vector<dbTech*>& techs, odb::dbSet<odb::dbTech> MIV_tech)
{
  _dist_factor = 1.0 / (double) techs[0]->getDbUnitsPerMicron();
  _area_factor = _dist_factor * _dist_factor;
  write3DTechBody(techs, MIV_tech);

  std::string work_dir  = config_3D_pdk_->getWorkspaceDir();
  std::string tracks_rel_path = config_3D_pdk_->getTracksfilePath();
  if (!work_dir.empty() && work_dir.back() != '/' && tracks_rel_path.front() != '/')
      work_dir += '/';

  std::string full_tracks_path = work_dir + tracks_rel_path;
  printf("Using 3D tracks file: %s\n", full_tracks_path.c_str());
  writeMakeTracksFile(full_tracks_path, techs);
  fmt::print(_out, "END LIBRARY\n");
}

void lefout::write3DLib(dbLib* lib, int tier_index)
{
  _dist_factor = 1.0 / (double) lib->getDbUnitsPerMicron();
  _area_factor = _dist_factor * _dist_factor;
  
  writeHeader(lib);
  write3DLibBody(lib, tier_index);
}

void lefout::write3DTechAndLib(std::vector<dbTech*>& techs, odb::dbSet<odb::dbTech> MIV_tech, std::vector<dbLib*>& libs)
{
  if (techs.empty() || libs.empty()) {
      logger_->warn(utl::ODB, 911, "No techs or libs provided for 3D LEF generation.");
      return;
  }

  _dist_factor = 1.0 / (double) techs[0]->getDbUnitsPerMicron();
  _area_factor = _dist_factor * _dist_factor;
  write3DTechBody(techs, MIV_tech);

  int tier_index = 0;
  for (auto* lib : libs) {
    write3DLibBody(lib, tier_index);
    tier_index++;
  }

  fmt::print(_out, "END LIBRARY\n");
}

void lefout::setConfig(odb::Config_3DPDK* config)
{
    config_3D_pdk_ = config;

    // Optional: log tier and integration summary for debugging
    if (logger_) {
        logger_->info(utl::ODB, 1200, "3D PDK configuration set in lefout writer.");
        logger_->info(utl::ODB, 1201, "Workspace directory: {}", config_3D_pdk_->getWorkspaceDir());
        logger_->info(utl::ODB, 1202, "Number of tiers: {}", config_3D_pdk_->getTiers().size());
    }
}

// ---------------- Helper Functions ---------------- //
bool lefout::isValidIntegrationTransition(const std::string& previous,
                                          const std::string& current)
{
    // Allowed transitions based on your example sequence:
    // 1st→2nd: F2F
    // 2nd→3rd: B2B
    // 3rd→4th: F2B
    // 4th→5th: F2F
    if ((previous == "F2F" && current == "B2B") ||
        (previous == "B2B" && current == "F2B") ||
        (previous == "F2B" && current == "F2F") ||
        (previous == "F2F" && current == "F2F")) { // allow consecutive F2F
        return true;
    }

    // Any other transition is invalid
    return false;
}

dbTech* lefout::findTechForTier(const std::vector<dbTech*>& techs,
                                const Config_3DPDK::TierInfo& tier)
{
    auto toLower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        return s;
    };

    std::string target_pdk = toLower(tier.pdk_name);

    for (auto* tech : techs) {
        if (!tech)
            continue;

        std::string tech_name = toLower(tech->getName());

        printf("Checking tech: %s for tier: %s (PDK: %s)\n",
               tech->getName().c_str(),
               tier.name.c_str(),
               tier.pdk_name.c_str());

        // Match if names are equal or one contains the other
        if (tech_name == target_pdk || tech_name.find(target_pdk) != std::string::npos) {
            return tech;
        }
    }

    logger_->warn(utl::ODB, 901,
                  "No tech found for PDK {} (Tier {})",
                  tier.pdk_name, tier.name);
    return nullptr;
}

void lefout::writeTierLayers(dbTech* tier_tech,
                             const Config_3DPDK::TierInfo& tier,
                             const std::vector<dbTechLayer*>& tier_layers,
                             size_t tier_idx,
                             const std::string& integration_type)
{
    if (tier_layers.empty()) return;

    // Write the tier layers in the correct order depending on the integration type
    if (integration_type == "F2F" || integration_type == "F2B") {
        // Normal order for bottom tier
        for (auto* layer : tier_layers) {
            write3DLayer(layer, tier_idx);
        }
    } else if (integration_type == "B2B") {
        // Reverse order for bottom tier
        for (auto it = tier_layers.rbegin(); it != tier_layers.rend(); ++it) {
            write3DLayer(*it, tier_idx);
        }
    } else {
        // Default order
        for (auto* layer : tier_layers) {
            write3DLayer(layer, tier_idx);
        }
    }
}

void lefout::writeGeneralTechInfo(dbTech* tech)
{
    assert(tech);

    if (tech->hasNoWireExtAtPin()) {
        fmt::print(_out, "NOWIREEXTENSIONATPIN {} ;\n", tech->getNoWireExtAtPin().getString());
    }
    if (tech->hasClearanceMeasure()) {
        fmt::print(_out, "CLEARANCEMEASURE {} ;\n", tech->getClearanceMeasure().getString());
    }
    if (tech->hasUseMinSpacingObs()) {
        fmt::print(_out, "USEMINSPACING OBS {} ;\n", tech->getUseMinSpacingObs().getString());
    }
    if (tech->hasUseMinSpacingPin()) {
        fmt::print(_out, "USEMINSPACING PIN {} ;\n", tech->getUseMinSpacingPin().getString());
    }
    if (tech->hasManufacturingGrid()) {
        fmt::print(_out, "MANUFACTURINGGRID {:.11g} ;\n", lefdist(tech->getManufacturingGrid()));
    }
}

void lefout::collectTierLayers(dbTech* tech, const Config_3DPDK::TierInfo& tier,
                               std::vector<dbTechLayer*>& tier_layers)
{
    dbSet<dbTechLayer> layers = tech->getLayers();
    bool start_collect = false;
    bool found_max = false;

    for (auto* layer : layers) {
        std::string lname = layer->getName();
        if (!start_collect) {
            if (lname == tier.min_metal_layer) {
                start_collect = true;
            } else {
                continue;
            }
        }

        if (start_collect) {
            tier_layers.push_back(layer);
            if (lname == tier.max_metal_layer) {
                found_max = true;
                break;
            }
        }
    }

    if (!found_max) {
        logger_->warn(utl::ODB, 2005,
                          "Tier {}: max metal layer {} not found in PDK {}",
                          tier.name, tier.max_metal_layer, tier.pdk_name);
    }
}

std::vector<dbTechLayer*> lefout::synthesizeMIVLayer(size_t tier_idx, odb::dbSet<odb::dbTech> MIV_tech)
{
  const auto& integration_stack = config_3D_pdk_->getIntegrationStack();
  std::vector<dbTechLayer*> miv_layers;
  dbTechLayer* miv_layer = nullptr;
  const auto& link = integration_stack[tier_idx];
  

  for (const auto& ic : link.interconnects) {
    for (odb::dbTech* tech : MIV_tech) {
      for (const auto& layer : tech->getLayers()) {
        printf("Checking MIV layer: %s for interconnect: %s\n",
               layer->getName().c_str(),
               ic.name.c_str());
        if (layer->getName() == ic.name && layer->getType() == dbTechLayerType::CUT) {
          miv_layer = layer;
          break;
        }
      }
    }
    miv_layers.push_back(miv_layer);
  }

  return miv_layers;
}

void lefout::writeMakeTracksFile(const std::string& filename, const std::vector<odb::dbTech*>& techs)
{
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open tracks file: " << filename << "\n";
        return;
    }

    for (size_t tier_idx = 0; tier_idx < techs.size(); ++tier_idx) {
        odb::dbTech* tech = techs[tier_idx];
        const auto& tier = config_3D_pdk_->getTiers()[tier_idx];

        // Iterate through layers from min_metal_layer to max_metal_layer
        bool in_metal_range = false;
        for (auto* layer : tech->getLayers()) {
            std::string lname = layer->getName();
            
            // Only process ROUTING layers
            if (layer->getType() != odb::dbTechLayerType::ROUTING)
                continue;

            if ((in_metal_range == false) && (lname == tier.min_metal_layer)) in_metal_range = true;
            if (!in_metal_range) continue;

            // Extract track info from LEF
            double x_pitch = lefdist(layer->getPitchX());      // or getPitch depending on dbTechLayer API
            double y_pitch = lefdist(layer->getPitchY());
            double x_offset = lefdist(layer->getOffsetX());   // if not available, use 0.0
            double y_offset = lefdist(layer->getOffsetY());   // if not available, use 0.0

            lname += "_" + config_3D_pdk_->getTiers()[tier_idx].name;
            ofs << "make_tracks " << lname
                << " -x_offset " << x_offset
                << " -x_pitch " << x_pitch
                << " -y_offset " << y_offset
                << " -y_pitch " << y_pitch << "\n";

            if (lname == tier.max_metal_layer) break;  // reached top layer
        }
    }

    ofs.close();
    std::cout << "Tracks file written to: " << filename << "\n";
}

