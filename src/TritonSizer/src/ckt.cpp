#include "sizer.h"
#include "ckt.h"
#include <tcl8.6/tcl.h>
#include <tcl8.6/tclDecls.h>
#include <cassert>
#include <cstdio>
#include <iostream>
#include "sta/ConcreteNetwork.hh"
#include "sta/PortExtCap.hh"
#include "sta/InputDrive.hh"
#include "sta/PatternMatch.hh"
#include "sta/PortDelay.hh"
#include "sta/Sdc.hh"
#define NUM_VTS 3
#define NUM_SIZES 10
#define SDC_FILE_POSTFIX ".sizer"
#define SDC_CONVERT_TCL "./SDC/convert_sdc.tcl"
#include "sta/Sta.hh"
#include "ConcreteParasitics.hh"
#include "ConcreteParasiticsPvt.hh"
using namespace std;
using namespace sta;

void skip(istream& is);

// Sort the list by leakage
//

bool compLeak(LibCellInfo* a, LibCellInfo* b) {
    return (a->leakagePower < b->leakagePower);
}

string Circuit::getFullPinName(PIN& pin) {
    string full_pin_name;
    if(pin.owner != UINT_MAX) {
        full_pin_name = g_cells[pin.owner].name + "/" + pin.name;
    }
    else {
        full_pin_name = pin.name;
    }
    return full_pin_name;
}

bool Circuit::isDontUse(string master) {
    for(int j = 0; j < _sizer->dontUseCell.size(); j++) {
        if(_sizer->dontUseCell[j] == master) {
            return true;
        }
    }
    return false;
}

void Circuit::InitData() {
    /// SDC
    //
    for(unsigned i = 0; i < _sizer->numModes; ++i) {
        string _clk_name;
        string _clk_port;
        double _clk_period;
        double _guardband;
        double _power_clk_period;

        map< unsigned, double > _inrtran, _inftran;
        map< unsigned, unsigned > _driverInPins;
        map< unsigned, unsigned > _driverOutPins;
        map< string, double > _indelays;
        map< string, double > _outdelays;

        _sizer->clk_name.push_back(_clk_name);
        _sizer->clk_port.push_back(_clk_port);
        _sizer->clk_period.push_back(_clk_period);
        _sizer->guardband.push_back(_guardband);
        _sizer->power_clk_period.push_back(_power_clk_period);

        _sizer->inrtran.push_back(_inrtran);
        _sizer->inftran.push_back(_inftran);
#ifdef DRIVER_CELL
        map< unsigned, string > _drivers;
        _sizer->drivers.push_back(_drivers);
        _sizer->driverInPins.push_back(_driverInPins);
        _sizer->driverOutPins.push_back(_driverOutPins);
#endif
        _sizer->indelays.push_back(_indelays);
        _sizer->outdelays.push_back(_outdelays);
    }

    // LIB
    //
    //
    for(unsigned i = 0; i < _sizer->numCorners; ++i) {
        vector< NET > _net;
        vector< LibCellTable* > _main_lib_cell_tables;            // MMMC
        map< string, LibCellInfo > _libs;                         // MMMC
        map< string, int > _node2id;                              // MMMC
        map< string, list< LibCellInfo* > > _func_lib_cell_list;  // MMMC
        double _maxTran = 0.0;

        _sizer->main_lib_cell_tables.push_back(_main_lib_cell_tables);
        _sizer->libs.push_back(_libs);
        _sizer->node2id.push_back(_node2id);
        _sizer->maxTran.push_back(_maxTran);
        g_nets.push_back(_net);
        _sizer->func_lib_cell_list.push_back(_func_lib_cell_list);
    }
}

void Circuit::Parser(string benchmark) {
    InitData();
    if(!_sizer->mmmcOn) {
        for(unsigned i = 0; i < _sizer->libLibs.size(); ++i) {
            cout << "Parsing lib...     " << _sizer->libLibs[i] << endl;
            if(_sizer->libLibPath != "") {
                lib_parser(_sizer->libLibPath + "/" + _sizer->libLibs[i]);
            }
            else {
                lib_parser(_sizer->libLibs[i]);
            }
        }
    }
    else {
        for(unsigned corner = 0; corner < _sizer->mmmcLibLists.size();
            ++corner) {
            for(unsigned j = 0; j < _sizer->mmmcLibLists[corner].size(); ++j) {
                cout << "Parsing lib...     " << _sizer->mmmcLibLists[corner][j]
                     << endl;
                if(_sizer->libLibPath != "") {
                    lib_parser(_sizer->libLibPath + "/" +
                                   _sizer->mmmcLibLists[corner][j],
                               corner);
                }
                else {
                    lib_parser(_sizer->mmmcLibLists[corner][j], corner);
                }
            }
            _sizer->func2id.clear();
        }
    }

    cout << "Load design... " << _sizer->benchname << endl;
    _sta = new sta::Sta;
    init_opensta(_sta);
    readDesign_opensta(_sta);
    // if(!_sizer->mmmcOn) {
    //     cout << "Parsing sdc...     " << _sizer->sdcFile << endl;
    //     sdc_parser(_sizer->sdcFile);
    // }
    // else {
    //     for(unsigned mode = 0; mode < _sizer->mmmcSdcList.size(); ++mode) {
    //         cout << "Parsing sdc...     " << _sizer->mmmcSdcList[mode] <<
    //         endl; sdc_parser(_sizer->mmmcSdcList[mode], mode);
    //     }
    // }
    assert(_sta);
    for(unsigned i = 0; i < g_pins.size(); ++i) {
        string cell_name = "NA";
        if(g_pins[i].owner != UINT_MAX) {
            cell_name = g_cells[g_pins[i].owner].name;
        }
        assert(g_pins[i].cap < 1e31);
        if(VERBOSE > 4)
            cout << "PINS --- " << i << " " << cell_name << "/"
                 << g_pins[i].name << " " << g_pins[i].lib_pin << endl;
    }

    if(VERBOSE > 4) {
        cout << "PI TEST----" << endl;
        for(unsigned mode = 0; mode < _sizer->numModes; ++mode) {
            for(unsigned i = 0; i < PIs.size(); ++i) {
                assert(g_pins[PIs[i]].name != "");
                // assert(_sizer->drivers[mode][PIs[i]] != "");
                cout << PIs[i] << " " << g_pins[PIs[i]].name << " ";
#ifdef DRIVER_CELL
                << _sizer->drivers[mode][PIs[i]] << " "
                << _sizer->driverInPins[mode][PIs[i]]
                << " "
#endif
                    cout
                << _sizer->inrtran[mode][PIs[i]] << " "
                << _sizer->inftran[mode][PIs[i]] << endl;
            }
        }
    }

    map< string, unsigned > check_map = generateLibCellTable();

    assignLibCellTables(check_map);
    assignMaxTrans();
    assignLibPinId();

    for(map< string, unsigned >::iterator it = check_map.begin();
        it != check_map.end(); ++it) {
        if(VERBOSE > 1)
            cout << "CHECK MAP " << it->first << "---" << it->second << endl;
    }

    _sizer->RuntimeLimit = 3600 * (3 + ceil(g_cells.size() / 40000.));

    for(unsigned i = 0; i < g_cells.size(); ++i) {
        _sizer->init_sizes.push_back(g_cells[i].type);

        list< string > pin_list;
        for(unsigned j = 0; j < g_cells[i].inpins.size(); ++j) {
            pin_list.push_back(getFullPinName(g_pins[g_cells[i].inpins[j]]));
            // cout << g_pins[g_cells[i].inpins[j]].name << endl;
        }
        pin_list.sort();

        g_cells[i].inpins.clear();

        for(std::list< string >::iterator it = pin_list.begin();
            it != pin_list.end(); it++) {
            // cout << "IN PIN LIST " <<   (*it) << endl;
            g_cells[i].inpins.push_back(pin2id[(*it)]);
        }

        list< string > pin_list2;
        for(unsigned j = 0; j < g_cells[i].outpins.size(); ++j) {
            pin_list2.push_back(getFullPinName(g_pins[g_cells[i].outpins[j]]));
        }
        pin_list2.sort();

        g_cells[i].outpins.clear();

        for(std::list< string >::iterator it = pin_list2.begin();
            it != pin_list2.end(); it++) {
            // cout << "OUT PIN LIST " <<   (*it) << endl;
            g_cells[i].outpins.push_back(pin2id[(*it)]);
        }
    }

    if(MINIMUM) {
        // reset to min size / vt
        cout << "Reset to minimum size / vt ... " << endl;
        for(unsigned i = 0; i < g_cells.size(); ++i) {
            LibCellInfo* new_lib_cell_info =
                _sizer->getLibCellInfo(g_cells[i].main_lib_cell_id, 0, s);
            if(new_lib_cell_info != NULL) {
                g_cells[i].type = new_lib_cell_info->name;
                g_cells[i].c_size = 0;
                g_cells[i].c_vtype = s;
                // cout << g_cells[i].name << " " << g_cells[i].type << " " <<
                // r_size(g_cells[i]) << " " << r_type(g_cells[i]) << endl;
                for(unsigned j = 0; j < g_cells[i].inpins.size(); j++)
                    g_pins[g_cells[i].inpins[j]].cap =
                        new_lib_cell_info
                            ->pins[g_pins[g_cells[i].inpins[j]].lib_pin]
                            .capacitance;
            }
        }
    }

    if(_sizer->mmmcOn && _sizer->numCorners > 1) {
        cout << "copy nets to other corner" << endl;
        for(unsigned corner = 1; corner < _sizer->numCorners; ++corner) {
            cout << "copy to corner " << corner << endl;
            for(unsigned i = 0; i < g_nets[0].size(); ++i) {
                g_nets[corner][i] = g_nets[0][i];
            }
        }
    }

    for(unsigned corner = 0; corner < _sizer->numCorners; ++corner) {
        if(!_sizer->noSPEF) {
            if(_sizer->mmmcOn)
                cout << "Parsing spef...    " << _sizer->mmmcSpefList[corner]
                     << " for corner " << corner << endl;
            else
                cout << "Parsing spef...    " << endl;

            readSpef_opensta(_sta);

            node2id.clear();
        }
        for(unsigned i = 0; i < g_nets[corner].size(); ++i) {
            g_nets[corner][i].cap =
                g_nets[corner][i].cap * 1e-12 / _sizer->cap_unit;

            if(VERBOSE > 4)
                cout << "CORNER " << corner << " NETS --- " << i << " "
                     << g_nets[corner][i].name << endl;

            vector< SUB_NODE >* subNodeVecPtr = &g_nets[corner][i].subNodeVec;
            std::vector< SUB_NODE >::iterator subNodeIter;
            for(subNodeIter = subNodeVecPtr->begin();
                subNodeIter != subNodeVecPtr->end(); ++subNodeIter) {
                subNodeIter->cap = subNodeIter->cap * 1e-12 / _sizer->cap_unit;
                for(unsigned int j = 0; j < subNodeIter->adj.size(); ++j) {
                    subNodeIter->res[j] =
                        subNodeIter->res[j] / _sizer->res_unit;
                }
            }
            list< string > pin_list2;
            for(unsigned j = 0; j < g_nets[corner][i].outpins.size(); ++j) {
                pin_list2.push_back(
                    getFullPinName(g_pins[g_nets[corner][i].outpins[j]]));
            }
            pin_list2.sort();

            g_nets[corner][i].outpins.clear();

            for(std::list< string >::iterator it = pin_list2.begin();
                it != pin_list2.end(); it++) {
                // cout << "OUT PIN LIST " <<   (*it) << endl;
                g_nets[corner][i].outpins.push_back(pin2id[(*it)]);
                // cout << "NETS --- " << i << " " << g_nets[corner][i].name <<
                // "OUT PIN LIST " <<   (*it) << endl;
            }
        }
    }
}

// pankit 01/25/2013 - assign lib_pin_id for a pin (doesnt change even when type
// or size id changes)
//
//
void Circuit::assignLibPinId() {
    unsigned corner = 0;
    vector< PIN >::iterator pin_iter;
    map< string, unsigned >::iterator temp_iter;

    for(pin_iter = g_pins.begin(); pin_iter != g_pins.end(); ++pin_iter) {
        PIN* pin = &(*pin_iter);
        if(pin->owner != UINT_MAX) {
            CELL& cell = g_cells[pin->owner];
            // cout << pin->name << " " << pin2id[cell.name+"/"+pin->name] << "
            // " << pin->owner << endl;
            LibCellInfo* lib_cell_info = _sizer->getLibCellInfo(cell);
            // Resize the rdelay/fdelay vector size
            pin->rdelay.resize(cell.outpins.size(), 0.0);
            pin->fdelay.resize(cell.outpins.size(), 0.0);
            pin->bb_checked_delay.resize(cell.outpins.size(), false);
            if(lib_cell_info == NULL) {
                continue;
            }
            if((temp_iter = lib_cell_info->lib_pin2id_map.find(pin->name)) !=
               lib_cell_info->lib_pin2id_map.end()) {
                pin->lib_pin = temp_iter->second;
                pin->cap = lib_cell_info->pins[pin->lib_pin].capacitance;
                assert(pin->cap < 1e31);
                // cout << "LIBPIN " << cell.name << "--" << pin->name <<"/" <<
                // pin->lib_pin << endl;
                // cout << lib_cell_info->pins[pin->lib_pin] ;
            }
        }
        else {
            // cout << pin->name << " " << pin2id[pin->name] << " " <<
            // pin->owner << endl;
        }
    }

    cout << "END OF PIN MAPPING" << endl;

    for(unsigned i = 0; i < g_cells.size(); ++i) {
        LibCellInfo* lib_cell_info = _sizer->getLibCellInfo(g_cells[i]);

        if(g_cells[i].outpins.size() != 0) {
            g_cells[i].outpin = g_cells[i].outpins[0];
        }

        string cellInst, master;
        // dont_touch cell
        master = g_cells[i].type;
        cellInst = g_cells[i].name;
        // if(VERBOSE >= 200)
        //     cout << "INST " << i << " " << cellInst << " " << master << endl;
        for(int j = 0; j < _sizer->dontTouchCell.size(); j++) {
            if(_sizer->dontTouchCell[j] == master) {
                g_cells[i].isDontTouch = true;
                if(VERBOSE >= 1)
                    cout << "LIST DONT TOUCH " << g_cells[i].name << " "
                         << g_cells[i].type << endl;
            }
        }

        for(int j = 0; j < _sizer->dontTouchInst.size(); j++) {
            if(_sizer->dontTouchInst[j] == cellInst) {
                g_cells[i].isDontTouch = true;
                if(VERBOSE >= 1)
                    cout << "LIST DONT TOUCH " << g_cells[i].name << " "
                         << g_cells[i].type << endl;
            }
        }

        if(lib_cell_info != NULL) {
            if(lib_cell_info->isSequential) {
                CELL& cell = g_cells[i];
                cell.isFF = true;

                if(NO_SEQ_OPT) {
                    cell.isDontTouch = true;
                }

                FFs.push_back(i);

                for(unsigned j = 0; j < cell.inpins.size(); ++j) {
                    PIN* pin = &g_pins[cell.inpins[j]];

                    if(lib_cell_info->pins[pin->lib_pin].isClock) {
                        string pin_name = cell.name + "/" + pin->name;
                        cell.clock_pin = pin2id[pin_name];
                    }
                    if(lib_cell_info->pins[pin->lib_pin].isData) {
                        string pin_name = cell.name + "/" + pin->name;
                        cell.data_pin = pin2id[pin_name];
                    }
                }

                if(g_cells[i].clock_pin == UINT_MAX) {
                    cout << "WARNING: " << cell.type
                         << " is a sequential cell, but has no clock pin "
                         << endl;
                    cell.isDontTouch = true;
                    continue;
                }

                if(g_cells[i].data_pin == UINT_MAX) {
                    cout << "WARNING: " << cell.type
                         << " is a sequential cell, but has no data pin "
                         << endl;
                    cell.isDontTouch = true;
                    continue;
                }

                unsigned pin_index = UINT_MAX;

                if(g_pins[g_cells[i].clock_pin].net != UINT_MAX) {
                    pin_index =
                        g_nets[corner][g_pins[g_cells[i].clock_pin].net].inpin;
                }
                else {
                    cout << "WARNING: no net for clock pin " << cell.type << " "
                         << cell.name << " "
                         << g_pins[g_cells[i].clock_pin].name << endl;
                }

                if(pin_index != UINT_MAX && NO_CLKBUF_OPT) {
                    vector< bool > visited;
                    visited.resize(g_pins.size());
                    for(unsigned k = 0; k < g_pins.size(); ++k) {
                        visited[k] = false;
                    }

                    while(!g_pins[pin_index].isPI) {
                        if(visited[pin_index]) {
                            cout << "The pin is already visited!" << endl;
                            break;
                        }

                        visited[pin_index] = true;

                        if(g_cells[g_pins[pin_index].owner].isFF) {
                            cout << "Found a sequential cell!" << endl;
                            break;
                        }
                        g_cells[g_pins[pin_index].owner].isClockCell = true;
                        g_cells[g_pins[pin_index].owner].isDontTouch = true;

                        if(g_cells[g_pins[pin_index].owner].inpins.size() ==
                           0) {
                            cout << "Input pin size is zero " << endl;
                            break;
                        }
                        else {
                            if(g_cells[g_pins[pin_index].owner].inpins[0] ==
                               UINT_MAX) {
                                cout << "First input pin is invalid " << endl;
                                break;
                            }
                            else if(g_pins[g_cells[g_pins[pin_index].owner]
                                               .inpins[0]]
                                        .net == UINT_MAX) {
                                cout << "Input net is invalid" << endl;
                                break;
                            }
                        }
                        if(VERBOSE >= 1)
                            cout << "CLOCK CELL DONT TOUCH "
                                 << g_cells[g_pins[pin_index].owner].name << " "
                                 << g_cells[g_pins[pin_index].owner].type << " "
                                 << g_nets
                                        [corner]
                                        [g_pins[g_cells[g_pins[pin_index].owner]
                                                    .inpins[0]]
                                             .net]
                                            .name
                                 << " "
                                 << g_nets
                                        [corner]
                                        [g_pins[g_cells[g_pins[pin_index].owner]
                                                    .inpins[0]]
                                             .net]
                                            .inpin
                                 << endl;

                        pin_index =
                            g_nets[corner]
                                  [g_pins[g_cells[g_pins[pin_index].owner]
                                              .inpins[0]]
                                       .net]
                                      .inpin;
                        if(pin_index == UINT_MAX) {
                            break;
                        }
                    }
                }
            }

            if(lib_cell_info->footprint == "0" ||
               lib_cell_info->footprint == "1") {
                g_cells[i].isDontTouch = true;
                if(VERBOSE >= 1)
                    cout << "TIE DONT TOUCH " << g_cells[i].name << " "
                         << g_cells[i].type << endl;
            }
        }
        else {
            if(VERBOSE >= 1)
                cout << "BLACKBOX INST " << i << " " << cellInst << " "
                     << master << endl;
            // treat blackboxes
            //
            for(unsigned j = 0; j < g_cells[i].inpins.size(); ++j) {
                g_pins[g_cells[i].inpins[j]].isPO = true;
                POs.push_back(g_cells[i].inpins[j]);
            }

            for(unsigned j = 0; j < g_cells[i].outpins.size(); ++j) {
                g_pins[g_cells[i].outpins[j]].isPI = true;
                PIs.push_back(g_cells[i].outpins[j]);
            }
        }
    }
}

void Circuit::assignMaxTrans() {
    vector< CELL >::iterator iter;
    for(iter = g_cells.begin(); iter != g_cells.end(); ++iter) {
        CELL* cell = &(*iter);

        for(unsigned i = 0; i < _sizer->numCorners; ++i) {
            LibCellInfo* lib_cell = _sizer->getLibCellInfo(*cell, i);

            if(lib_cell == NULL) {
                cell->max_tran.push_back(_sizer->maxTran[i]);
            }
            else {
                if(lib_cell->max_tran != DBL_MAX) {
                    cell->max_tran.push_back(lib_cell->max_tran);
                }
                else {
                    cell->max_tran.push_back(_sizer->maxTran[i]);
                }
            }
        }
    }
}

// pankit 01/23/2013
//
void Circuit::assignLibCellTables(map< string, unsigned > check_map) {
    vector< CELL >::iterator iter;
    map< string, unsigned >::iterator temp_it;
    for(temp_it = check_map.begin(); temp_it != check_map.end(); ++temp_it) {
        // cout << "CHECK MAP " << temp_it->first << " " << temp_it->second <<
        // endl;
    }

    for(iter = g_cells.begin(); iter != g_cells.end(); ++iter) {
        CELL* cell = &(*iter);

        LibCellInfo* lib_cell = _sizer->getLibCellInfo(*cell);

        if(lib_cell == NULL) {
            continue;
        }

        string main_cell_type = lib_cell->footprint;
        cell->c_size = lib_cell->c_size;

        if((temp_it = check_map.find(main_cell_type)) != check_map.end()) {
            cell->main_lib_cell_id = (int)temp_it->second;
            // cout << "CELL TABLE " << cell->name << " " << cell->type << " "
            // << main_cell_type << " " << cell->main_lib_cell_id << " " <<
            // cell->c_vtype << " " << cell->c_size << endl;
        }
    }
}

// pankit 01/22/2013 - generate a 2D table for each type excluding the vtype and
// size.
// For example for a cell ms01XZZ will belong to the table for ms01 having size
// as rows and
// vtype as colums. X is the vtype and ZZ is the column
//
map< string, unsigned > Circuit::generateLibCellTable() {
    map< string, LibCellInfo >::iterator iter;
    // unsigned count = 0;
    map< string, unsigned > check_map;

    for(unsigned corner = 0; corner < _sizer->numCorners; ++corner) {
        check_map.clear();
        for(iter = _sizer->libs[corner].begin();
            iter != _sizer->libs[corner].end(); ++iter) {
            string main_cell_type = (iter->second).footprint;
            if(VERBOSE >= 4)
                report_cell(iter->second);

            if(main_cell_type == "NA") {
                continue;
            }

            if(check_map.find(main_cell_type) == check_map.end()) {
                LibCellTable* main_lib_cell_table = new LibCellTable;
                main_lib_cell_table->name = main_cell_type;
                createLibCellTable(*main_lib_cell_table, corner);
                check_map[main_cell_type] =
                    _sizer->main_lib_cell_tables[corner].size();
                _sizer->main_lib_cell_tables[corner].push_back(
                    main_lib_cell_table);
            }
        }
    }

    return check_map;
}

// may have a bug
// pankit 01/22/2013
void Circuit::createLibCellTable(LibCellTable& lib_cell_table,
                                 unsigned corner) {
    string type = lib_cell_table.name;

    list< LibCellInfo* >& candidate_list =
        _sizer->func_lib_cell_list[corner][type];

    // 如果有多个高vt呢,根据leakage顺序进行排序。
    // list size first
    for(std::list< LibCellInfo* >::iterator it = candidate_list.begin();
        it != candidate_list.end(); it++) {
        // slowest vt first
        // if((*it)->c_vtype != 0) {
        //     continue;
        // }

        vector< LibCellInfo* > lib_size_table;

        (*it)->c_size = lib_cell_table.lib_vt_size_table.size();
        lib_size_table.push_back((*it));
        // lib_size_table.resize(sizeof(LibCellInfo*)*_sizer->numVt);
        // lib_size_table.resize(_sizer->numVt); // ?

        lib_cell_table.lib_vt_size_table.push_back(lib_size_table);
    }
    // cout << "-------------------" << endl;

    // add vt
    assert(lib_cell_table.lib_vt_size_table.size() > 0);
#if 0
    for(unsigned i = 0; i < lib_cell_table.lib_vt_size_table.size(); ++i) {
        LibCellInfo* lib_cell = lib_cell_table.lib_vt_size_table[i][0];

        string lib_cell_name = lib_cell->name;

        if(_sizer->numVt > 1) {
            size_t start = lib_cell_name.find(_sizer->suffixHVT.c_str());
            lib_cell_name.erase(start, _sizer->suffixHVT.size());
        }

        for(std::list< LibCellInfo* >::iterator it = candidate_list.begin();
            it != candidate_list.end(); it++) {
            string newCellName = (*it)->name;
            unsigned vt = 0;

            if(_sizer->numVt == 3) {
                if(newCellName.find(_sizer->suffixHVT.c_str()) !=
                   std::string::npos) {
                    size_t start = newCellName.find(_sizer->suffixHVT.c_str());
                    newCellName.erase(start, _sizer->suffixHVT.size());
                    vt = 0;
                }
                else if(newCellName.find(_sizer->suffixLVT.c_str()) !=
                        std::string::npos) {
                    size_t start = newCellName.find(_sizer->suffixLVT.c_str());
                    newCellName.erase(start, _sizer->suffixLVT.size());
                    vt = 2;
                }
                else if(newCellName.find(_sizer->suffixNVT.c_str()) !=
                        std::string::npos) {
                    size_t start = newCellName.find(_sizer->suffixNVT.c_str());
                    newCellName.erase(start, _sizer->suffixNVT.size());
                    vt = 1;
                }
                else {
                    if(_sizer->suffixLVT.c_str() == "") {
                        vt = 2;
                    }
                    else if(_sizer->suffixNVT.c_str() == "") {
                        vt = 1;
                    }
                    else if(_sizer->suffixHVT.c_str() == "") {
                        vt = 0;
                    }
                }
            }
            else if(_sizer->numVt == 2) {
                if(newCellName.find(_sizer->suffixHVT.c_str()) !=
                   std::string::npos) {
                    size_t start = newCellName.find(_sizer->suffixHVT.c_str());
                    newCellName.erase(start, _sizer->suffixHVT.size());
                    vt = 0;
                }
                else if(newCellName.find(_sizer->suffixNVT.c_str()) !=
                        std::string::npos) {
                    size_t start = newCellName.find(_sizer->suffixNVT.c_str());
                    newCellName.erase(start, _sizer->suffixNVT.size());
                    vt = 1;
                }
                else {
                    if(_sizer->suffixNVT.c_str() == "") {
                        vt = 1;
                    }
                    else if(_sizer->suffixHVT.c_str() == "") {
                        vt = 0;
                    }
                }
            }
            else {
                continue;
            }

            if(newCellName != lib_cell_name) {
                continue;
            }

            if((*it)->c_vtype == 0) {
                continue;
            }

            (*it)->c_size = lib_cell->c_size;
            // lib_cell_table.lib_vt_size_table[i].push_back(&(*it));
            lib_cell_table.lib_vt_size_table[i][vt] = (*it);
        }
    }
#endif
    // test
    if(VERBOSE >= 1) {
        cout << "LIB CELL TABLE " << type
             << " #SIZE = " << lib_cell_table.lib_vt_size_table.size()
             << " #VT = ";
        if(lib_cell_table.lib_vt_size_table.size() != 0) {
            cout << lib_cell_table.lib_vt_size_table[0].size() << endl;
        }
        else {
            cout << " 0 " << endl;
        }

        for(unsigned i = 0; i < lib_cell_table.lib_vt_size_table.size(); ++i) {
            for(unsigned j = 0; j < lib_cell_table.lib_vt_size_table[i].size();
                ++j) {
                if(lib_cell_table.lib_vt_size_table[i][j] != NULL) {
                    cout << "SIZE/VT " << i << "/" << j << " : "
                         << lib_cell_table.lib_vt_size_table[i][j]->name
                         << endl;
                }
            }
        }
    }
}

void Circuit::Print_Stats() {
    unsigned corner = 0;
    cout << endl << "--------------------------------------------" << endl;
    cout << "NumCells  : " << setw(7) << g_cells.size() << "\t";
    cout << "NumFFs    : " << setw(7) << FFs.size() << endl;
    cout << "NumNets   : " << setw(7) << g_nets[corner].size() << "\t";
    cout << "NumPins   : " << setw(7) << g_pins.size() << endl;
    cout << "NumPIs    : " << setw(7) << PIs.size() << "\t";
    cout << "NumPOs    : " << setw(7) << POs.size() << endl;
    cout << "NumPCells : " << setw(7) << g_phy_cells.size() << endl;
    cout << "--------------------------------------------" << endl << endl;
    cout << "Sanity check..." << endl;

    for(unsigned i = 0; i < g_nets[corner].size(); i++) {
        if(g_nets[corner][i].inpin == UINT_MAX) {
            cout << "ERROR: floating input in net " << g_nets[corner][i].name
                 << endl;
        }
        for(unsigned j = 0; j < g_nets[corner][i].outpins.size(); j++) {
            if(g_nets[corner][i].outpins[j] == UINT_MAX) {
                cout << "ERROR: floating output in net "
                     << g_nets[corner][i].name << endl;
            }
        }
    }

    cout << " done." << endl;

    for(unsigned mode = 0; mode < _sizer->numModes; ++mode) {
        if(_sizer->power_clk_period[mode] == 0.0) {
            _sizer->power_clk_period[mode] = _sizer->clk_period[mode];
        }

        if(GUARD_BAND != 0) {
            _sizer->clk_period[mode] += GUARD_BAND;
        }
        else {
            _sizer->clk_period[mode] *= (100 - DIFFICULTY) * 0.01;
        }
        cout << "At mode " << mode << ", clock " << _sizer->clk_name[mode]
             << " connected to port " << _sizer->clk_port[mode]
             << " has period " << _sizer->clk_period[mode] << endl;
    }
    for(unsigned corner = 0; corner < _sizer->numCorners; ++corner) {
        if(MAX_TRAN_CONST != 0) {
            _sizer->maxTran[corner] = MAX_TRAN_CONST;
        }
        cout << "At corner " << corner
             << ", the default max transition defined is "
             << _sizer->maxTran[corner] << endl;
    }
}

void Circuit::sdc_converter(string filename) {
    char Commands[250];
    sprintf(Commands, "%s %s", SDC_CONVERT_TCL, filename.c_str());
    cout << Commands << endl;
    ;
    system(Commands);
}

// Example function that uses Circuit class to parse the given ISPD-12 sdc
// file. The extracted data is simply printed out in this example.
void Circuit::sdc_parser(string filename, unsigned mode) {
    char sdc_filename[250];

    sprintf(sdc_filename, "%s", filename.c_str());
    cout << "SDC " << sdc_filename << endl;

    // sdc_converter(filename);
    is.open(filename);
    bool valid = read_clock(_sizer->clk_name[mode], _sizer->clk_port[mode],
                            _sizer->clk_period[mode]);
    // assert(valid);

    // TODO:
    //  _sizer->indelays[mode][portName] = delay;

    // cout << "Driver Info " << portName << " " << driverSize << " "
    //         << inputTransitionRise << " " << inputTransitionFall
    //         << endl;

    // unsigned corner = 0;
    // LibCellInfo& lib_cell =
    //     _sizer->libs[corner].find(driverSize)->second;

    // //         // JLTimingArc: add driverOutPin info
    // std::map< unsigned, LibPinInfo >::iterator it;
    // for(it = lib_cell.pins.begin(); it != lib_cell.pins.end(); ++it)
    // {
    //     if((it->second).isInput == true) {
    //         driverInPin = lib_cell.lib_pin2id_map[(it->second).name];
    //     }
    //     if((it->second).isOutput == true) {
    //         driverOutPin =
    //         lib_cell.lib_pin2id_map[(it->second).name];
    //     }
    // }

    // _sizer->inrtran[mode].insert(std::pair< unsigned, double >(
    //     pin2id[portName], inputTransitionRise));
    // _sizer->inftran[mode].insert(std::pair< unsigned, double >(
    //     pin2id[portName], inputTransitionFall));
    // _sizer->outdelays[mode][portName] = delay;
    // g_pins[pin2id[portName]].cap = load;
    is.close();
}

void Circuit::lib_parser(string filename, unsigned corner) {
    is.open(filename.c_str());
    if(!is.good()) {
        cout << "ERROR: failed to open library " << filename << endl;
        exit(0);
    }
    cout << "Reading .lib file " << filename << " for corner " << corner
         << endl;

    LibInfo lib;

    lib.name = read_lib_name(is);
    cout << "Library name = " << lib.name << endl;

    read_head_info(is, lib, corner);

    if(_sizer->maxTran[corner] == 0.0) {
        _sizer->maxTran[corner] = 0.5;
    }

    is.seekg(0, is.beg);

    vector< string > tokens;
    vector< LibCellInfo > cells;
    while(!is.eof()) {
        read_line_as_tokens(is, tokens);
        // Read table templates
        if(tokens.size() == 2 && (tokens[0] == "power_lut_template" ||
                                  tokens[0] == "lu_table_template")) {
            LibTableTempl templ;
            templ.name = tokens[1];
            // cout << "Reading table template " << templ.name << endl;
            _begin_read_templ_info(is, templ);
            lib.templs.insert(
                std::pair< string, LibTableTempl >(templ.name, templ));
        }

        // Read cell info
        if(tokens.size() == 2 && tokens[0] == "cell") {
            LibCellInfo cell;
            // cell.dontUse = false;
            cell.dontTouch = false;
            cell.libname = lib.name;
            cell.name = tokens[1];
            string cellName = cell.name;
            cell.dontUse = isDontUse(cell.name);
            // cout << "Reading cell " << cell.name << endl;
            _begin_read_cell_info(is, cell, lib);

            // Find Vt of the cell
            if(_sizer->numVt == 3) {
                if(cellName.find(_sizer->suffixLVT) != std::string::npos) {
                    cell.c_vtype = f;
                }
                else if(cellName.find(_sizer->suffixHVT) != std::string::npos) {
                    cell.c_vtype = s;
                }
                else {
                    cell.c_vtype = m;
                }
            }
            else if(_sizer->numVt == 2) {
                if(cellName.find(_sizer->suffixHVT) != std::string::npos) {
                    cell.c_vtype = s;
                }
                else {
                    cell.c_vtype = m;
                }
            }
            else {
                cell.c_vtype = s;
            }
            cells.push_back(cell);

            if(VERBOSE > 1) {
                report_cell(cell);
            }
        }
    }

    _sizer->LIBs.insert(pair< string, LibInfo >(lib.name, lib));

    // JLPWR
    _sizer->sw_adj =
        lib.sw_power_unit * lib.volt * lib.volt / lib.time_unit / 2e-3;
    _sizer->res_unit = 1e3;
    _sizer->cap_unit = lib.cap_unit;
    _sizer->time_unit = lib.time_unit;

    LibCellInfo cur_cell;

    for(unsigned i = 0; i < cells.size(); i++) {
        cur_cell = cells[i];
        _sizer->libs[corner].insert(
            pair< string, LibCellInfo >(cur_cell.name, cur_cell));
    }
    for(unsigned i = 0; i < cells.size(); i++) {
        cur_cell = cells[i];
        map< string, unsigned >::iterator temp_iter;

        if((temp_iter = _sizer->func2id.find(cur_cell.footprint)) !=
           _sizer->func2id.end()) {
            if(!cur_cell.dontUse) {
                _sizer->func_lib_cell_list[corner][cur_cell.footprint]
                    .push_back(
                        &_sizer->libs[corner].find(cur_cell.name)->second);
            }
        }
        else {
            _sizer->func2id.insert(pair< string, unsigned >(
                cur_cell.footprint, _sizer->func2id.size()));
            if(!cur_cell.dontUse) {
                list< LibCellInfo* > cell_list;
                cell_list.push_back(
                    &_sizer->libs[corner].find(cur_cell.name)->second);
                _sizer->func_lib_cell_list[corner].insert(
                    std::pair< string, list< LibCellInfo* > >(
                        cur_cell.footprint, cell_list));
            }
        }
    }

    std::map< string, list< LibCellInfo* > >::iterator it;
    for(it = _sizer->func_lib_cell_list[corner].begin();
        it != _sizer->func_lib_cell_list[corner].end(); ++it) {
        (it->second).sort(compLeak);
    }

    is.close();
}

string Circuit::read_lib_name(istream& is) {
    bool finishedReading = false;

    std::vector< string > tokens;

    while(!finishedReading) {
        read_line_as_tokens_chk(is, tokens);

        if(tokens.size() == 2 && tokens[0] == "library") {
            return tokens[1];
        }
    }
}

// Read unit, default_max_transition
void Circuit::read_head_info(istream& is, LibInfo& lib, unsigned corner) {
    unsigned cnt = 0;
    lib.max_transition = 0.0;

    std::vector< string > tokens;

    while(cnt < 7) {
        read_line_as_tokens_chk(is, tokens);

        if(tokens.size() == 0) {
            continue;
        }

        if(tokens.size() == 2 && tokens[0] == "cell") {
            break;
        }

        if(tokens.size() == 2 && tokens[0] == "default_max_transition") {
            // lib.max_transition = std::atof(tokens[1].c_str()) * lib.time_unit
            // / 1e-9;
            lib.max_transition = std::atof(tokens[1].c_str());
            // lib.max_transition = std::atof(tokens[1].c_str());
            // cout << "Default maximum transition is " << lib.max_transition <<
            // " ns" << endl;
            cout << "Default maximum transition is "
                 << lib.max_transition * lib.time_unit << endl;
            // if ( _sizer->maxTran == 0.0 || _sizer->maxTran >
            // lib.max_transition ) {
            if(_sizer->maxTran[corner] == 0.0 ||
               _sizer->maxTran[corner] < lib.max_transition) {
                _sizer->maxTran[corner] = lib.max_transition;
            }
            ++cnt;
        }

        if(tokens.size() == 2 && tokens[0] == "voltage") {
            lib.volt = std::atof(tokens[1].c_str());
            cout << "Default voltage = " << lib.volt << endl;
            ++cnt;
        }

        if(tokens.size() == 2 && tokens[0] == "time_unit") {
            string temp = tokens[1];
            if(temp == "1ns") {
                lib.time_unit = 1e-9;
            }
            else if(temp == "1ps") {
                lib.time_unit = 1e-12;
            }
            else {
                lib.time_unit = 1e-9;
            }
            cout << "Time unit = " << lib.time_unit << "s" << endl;
            ++cnt;
        }

        if(tokens.size() == 2 && tokens[0] == "voltage_unit") {
            string temp = tokens[1];
            if(temp == "1V") {
                lib.voltage_unit = 1.0;
            }
            else if(temp == "1mV") {
                lib.voltage_unit = 1e-3;
            }
            else {
                lib.voltage_unit = 1.0;
            }
            cout << "Voltage unit = " << lib.voltage_unit << "V" << endl;
            ++cnt;
        }

        if(tokens.size() == 2 && tokens[0] == "current_unit") {
            string temp = tokens[1];
            if(temp == "1A") {
                lib.current_unit = 1.0;
            }
            else if(temp == "1mA") {
                lib.current_unit = 1e-3;
            }
            else {
                lib.current_unit = 1e-3;
            }
            cout << "Current unit = " << lib.current_unit << "A" << endl;
            ++cnt;
        }

        if(tokens[0] == "capacitive_load_unit") {
            string temp = "init";
            if(tokens.size() == 3) {
                temp = tokens[2];
            }
            else if(tokens.size() == 2) {
                read_line_as_tokens_chk(is, tokens);
                if(tokens.size() == 1) {
                    temp = tokens[0];
                }
            }
            if(temp == "nF" || temp == "nf") {
                lib.cap_unit = 1e-9;
            }
            else if(temp == "pF" || temp == "pf") {
                lib.cap_unit = 1e-12;
            }
            else if(temp == "fF" || temp == "ff") {
                lib.cap_unit = 1e-15;
            }
            else {
                lib.cap_unit = 1e-12;
            }
            cout << "Cap unit = " << lib.cap_unit << "F" << endl;
            if(temp == "init") {
                cout << "[Warning] There is no cap unit info !" << endl;
            }
            ++cnt;
        }

        if(tokens.size() == 2 && tokens[0] == "leakage_power_unit") {
            string temp = tokens[1];
            if(temp == "1W") {
                lib.leak_power_unit = 1.0;
            }
            else if(temp == "1mW") {
                lib.leak_power_unit = 1e-3;
            }
            else if(temp == "1uW") {
                lib.leak_power_unit = 1e-6;
            }
            else if(temp == "1nW") {
                lib.leak_power_unit = 1e-9;
            }
            else if(temp == "1pW") {
                lib.leak_power_unit = 1e-12;
            }
            else {
                lib.leak_power_unit = 1e-9;
            }
            cout << "Leakage power unit = " << lib.leak_power_unit << "W"
                 << endl;
            ++cnt;
        }
    }
    lib.int_power_unit =
        lib.voltage_unit * lib.voltage_unit * lib.cap_unit / lib.time_unit;
    cout << "Internal power unit = " << lib.int_power_unit << "W" << endl;
    lib.sw_power_unit = lib.voltage_unit * lib.voltage_unit * lib.cap_unit;
    cout << "Switching power unit = " << lib.sw_power_unit << "W" << endl;
}

void Circuit::_begin_read_templ_info(istream& is, LibTableTempl& templ) {
    bool finishedReading = false;

    int check = 1;

    vector< double > temp1;
    vector< double > temp2;

    temp1.clear();
    temp2.clear();

    std::vector< string > tokens;

    templ.loadFirst = false;
    templ.tranFirst = false;

    while(!finishedReading) {
        check += read_line_as_tokens_chk(is, tokens);

        if(check == 0) {
            // cout << "Finish reading lut template " << templ.name << endl;
            finishedReading = true;
            if(templ.tranFirst) {
                for(unsigned i = 0; i < temp1.size(); ++i) {
                    templ.transitionIndices.push_back(temp1[i]);
                }
                for(unsigned i = 0; i < temp2.size(); ++i) {
                    templ.loadIndices.push_back(temp2[i]);
                }
            }
            if(templ.loadFirst) {
                for(unsigned i = 0; i < temp1.size(); ++i) {
                    templ.loadIndices.push_back(temp1[i]);
                }
                for(unsigned i = 0; i < temp2.size(); ++i) {
                    templ.transitionIndices.push_back(temp2[i]);
                }
            }
        }

        if(tokens.size() == 0) {
            continue;
        }

        if(tokens.size() > 2 && tokens[0] == "index_1") {
            for(unsigned i = 1; i < tokens.size(); ++i) {
                temp1.push_back(atof(tokens[i].c_str()));
            }
        }

        if(tokens.size() > 2 && tokens[0] == "index_2") {
            for(unsigned i = 1; i < tokens.size(); ++i) {
                temp2.push_back(atof(tokens[i].c_str()));
            }
        }

        if(tokens[0] == "variable_1") {
            std::size_t found;
            found = tokens[1].find("transition");
            if(found != std::string::npos) {
                templ.tranFirst = true;
            }
            found = tokens[1].find("capacitance");
            if(found != std::string::npos) {
                templ.loadFirst = true;
            }
        }
    }
}

string Circuit::_begin_read_power_info(istream& is, string toPin,
                                       LibPowerInfo& power, LibInfo lib) {
    power.toPin = toPin;

    bool finishedReading = false;

    int check = 1;
    string related_pin;
    string pg_pin;
    std::vector< string > tokens;

    power.risePower.tableVals.clear();
    power.fallPower.tableVals.clear();

    while(!finishedReading) {
        check += read_line_as_tokens_chk(is, tokens);

        if(check == 0) {
            finishedReading = true;
        }

        if(tokens.size() == 0) {
            continue;
        }
        if(tokens[0] == "rise_power") {
            LibLUT lut;
            lut.templ = tokens[1];
            add_pg_pin(power, pg_pin);
            if(lut.templ == "scalar") {
                skip(is);
                --check;
            }
            else {
                _begin_read_lut(is, lut, "power", lib);
                update_lut(power.risePower, lut);
            }
        }
        else if(tokens[0] == "fall_power") {
            LibLUT lut;
            lut.templ = tokens[1];
            if(lut.templ == "scalar") {
                skip(is);
                --check;
            }
            else {
                _begin_read_lut(is, lut, "power", lib);
                update_lut(power.fallPower, lut);
            }
        }
        else if(tokens[0] == "related_pin") {
            related_pin = tokens[1];
        }
        else if(tokens[0] == "related_pg_pin") {
            pg_pin = tokens[1];
        }
        else if(tokens[0] == "when") {
            power.isFunc = true;
        }
    }
    power.relatedPin = related_pin;
    return related_pin;
}

void Circuit::average_lut(LibLUT& lut, int num) {
    if(num <= 0) {
        return;
    }
    for(unsigned i = 0; i < lut.loadIndices.size(); ++i) {
        for(unsigned j = 0; j < lut.transitionIndices.size(); ++j) {
            lut.tableVals[i][j] /= double(num);
        }
    }
}

// updated the old function
void Circuit::_begin_read_lut(istream& is, LibLUT& lut, string type,
                              LibInfo lib) {
    std::vector< string > tokens;

    bool flag;

    LibTableTempl& templ = lib.templs[lut.templ];
    // cout << "Table template name " << templ.name << endl;

    // Read indices
    unsigned size1 = 0, size2 = 0, index_num = 0;

    tokens.push_back("init");

    while(tokens[0] != "values") {
        read_line_as_tokens_chk(is, tokens);
        // Index 1
        if(tokens[0] == "index_1" && tokens.size() > 2) {
            size1 = tokens.size() - 1;
            if(templ.tranFirst) {
                lut.transitionIndices.resize(size1);
                for(unsigned i = 0; i < tokens.size() - 1; ++i) {
                    lut.transitionIndices[i] = atof(tokens[i + 1].c_str());
                }
            }
            else if(templ.loadFirst) {
                lut.loadIndices.resize(size1);
                for(unsigned i = 0; i < tokens.size() - 1; ++i) {
                    lut.loadIndices[i] = atof(tokens[i + 1].c_str());
                }
            }
            ++index_num;
        }
        // Index 2
        if(tokens[0] == "index_2" && tokens.size() > 2) {
            size2 = tokens.size() - 1;
            if(templ.tranFirst) {
                lut.loadIndices.resize(size2);
                for(unsigned i = 0; i < tokens.size() - 1; ++i) {
                    lut.loadIndices[i] = atof(tokens[i + 1].c_str());
                }
            }
            else if(templ.loadFirst) {
                lut.transitionIndices.resize(size2);
                for(unsigned i = 0; i < tokens.size() - 1; ++i) {
                    lut.transitionIndices[i] = atof(tokens[i + 1].c_str());
                }
            }
            ++index_num;
        }
    }

    // Use indices from templates
    if(index_num == 0) {
        lut.transitionIndices.clear();
        for(unsigned i = 0; i < templ.transitionIndices.size(); ++i) {
            lut.transitionIndices.push_back(templ.transitionIndices[i]);
        }
        lut.loadIndices.clear();
        for(unsigned i = 0; i < templ.loadIndices.size(); ++i) {
            lut.loadIndices.push_back(templ.loadIndices[i]);
        }
        if(templ.loadFirst) {
            size1 = templ.loadIndices.size();
            size2 = templ.transitionIndices.size();
        }
        else if(templ.tranFirst) {
            size1 = templ.transitionIndices.size();
            size2 = templ.loadIndices.size();
        }
    }

    if(tokens.size() == 1) {
        // flag indicates the "value" takes one line
        flag = true;
    }
    else {
        // if "values" is on the same line with tableVals, revise the vector
        tokens.erase(tokens.begin());
    }

    double ratio = 1;
    if(type == "power") {
        // Normalize the power unit to mW
        ratio = lib.int_power_unit / 1e-3;
    }
    else if(type == "timing") {
        // Normalize the timing unit to ns
        // ratio = lib.time_unit / 1e-9;
    }

    if(lut.transitionIndices.empty() || lut.loadIndices.empty()) {
        if(flag) {
            read_line_as_tokens_chk(is, tokens);
        }
        vector< double > tmpline;
        for(unsigned i = 0; i < tokens.size(); ++i) {
            tmpline.push_back(atof(tokens[i].c_str()) * ratio);
        }
        if(lut.loadIndices.empty()) {
            lut.loadIndices.push_back(0);
            lut.loadIndices.push_back(100);
            lut.tableVals.push_back(tmpline);
            lut.tableVals.push_back(tmpline);
        }
        else {
            lut.transitionIndices.push_back(0);
            lut.transitionIndices.push_back(100);
            lut.tableVals.resize(tmpline.size());
            for(unsigned i = 0; i < lut.loadIndices.size(); ++i) {
                lut.tableVals[i].resize(2);
            }
            for(unsigned i = 0; i < lut.loadIndices.size(); ++i) {
                lut.tableVals[i][0] = tmpline[i];
                lut.tableVals[i][1] = tmpline[i];
            }
        }
        return;
    }

    if(templ.loadFirst) {
        lut.tableVals.resize(size1);
        for(unsigned i = 0; i < size1; ++i) {
            if(flag || i != 0) {
                read_line_as_tokens_chk(is, tokens);
            }
            lut.tableVals[i].resize(size2);
            for(unsigned j = 0; j < size2; ++j) {
                lut.tableVals[i][j] = atof(tokens[j].c_str()) * ratio;
            }
        }
    }
    else if(templ.tranFirst) {
        // Change table so that index1 = load and index2 = tran
        lut.tableVals.resize(size2);
        for(unsigned j = 0; j < size2; ++j) {
            lut.tableVals[j].resize(size1);
        }
        for(unsigned i = 0; i < size1; ++i) {
            if(flag || i != 0) {
                read_line_as_tokens_chk(is, tokens);
            }
            for(unsigned j = 0; j < size2; ++j) {
                lut.tableVals[j][i] = atof(tokens[j].c_str()) * ratio;
            }
        }
    }
    // cout << "Load index size = " << lut.loadIndices.size();
    // cout << " Tran index size = " << lut.transitionIndices.size() << endl;
    // cout << "LUT size = " << lut.tableVals.size() << endl;
}

void Circuit::add_pg_pin(LibPowerInfo& powerTables, string pg_pin) {
    for(unsigned i = 0; i < powerTables.pgPins.size(); ++i) {
        if(powerTables.pgPins[i] == pg_pin) {
            return;
        }
    }
    powerTables.pgPins.push_back(pg_pin);
}

void Circuit::update_lut(LibLUT& toLut, LibLUT fromLut) {
    if(toLut.tableVals.empty()) {
        toLut.templ = fromLut.templ;
        toLut.loadIndices = fromLut.loadIndices;
        toLut.transitionIndices = fromLut.transitionIndices;
        toLut.tableVals = fromLut.tableVals;
    }
    else {
        for(unsigned i = 0; i < toLut.loadIndices.size(); ++i) {
            for(unsigned j = 0; j < toLut.transitionIndices.size(); ++j) {
                toLut.tableVals[i][j] += fromLut.tableVals[i][j];
            }
        }
    }
}

void Circuit::report_cell(LibCellInfo cell) {
    cout << "Library cell " << cell.name << ": " << endl;

    cout << "Vt: " << cell.c_vtype << endl;
    cout << "Footprint: " << cell.footprint << endl;
    cout << "Leakage power: " << cell.leakagePower << endl;
    cout << "Area: " << cell.area << endl;
    cout << "Sequential? " << (cell.isSequential ? "yes" : "no") << endl;
    cout << "Dont-touch? " << (cell.dontTouch ? "yes" : "no") << endl;

    cout << "Cell has " << cell.pins.size() << " pins: " << endl;
    std::map< unsigned, LibPinInfo >::iterator it;
    for(it = cell.pins.begin(); it != cell.pins.end(); ++it) {
        cout << it->second << endl;
    }

    cout << "Cell has " << cell.timingArcs.size() << " timing arcs: " << endl;
    std::map< unsigned, LibTimingInfo >::iterator ite;
    for(ite = cell.timingArcs.begin(); ite != cell.timingArcs.end(); ++ite) {
        cout << ite->second << endl;
    }
    cout << "Cell has " << cell.powerTables.size() << " power tables: " << endl;
    std::map< unsigned, LibPowerInfo >::iterator iter;
    for(iter = cell.powerTables.begin(); iter != cell.powerTables.end();
        ++iter) {
        cout << iter->second << endl;
    }

    cout << "End of cell " << cell.name << endl << endl;
}

bool is_special_char(char c) {
    // static const char specialChars[] = {'(', ')', ',', ':', ';', '/', '#',
    // '[', ']', '{', '}', '*', '\"', '\\'} ;
    static const char specialChars[] = {',', '\"', '\\', ';', '(', ')', ':',
                                        '{', '}',  '/',  '[', ']', '*', '#'};

    for(unsigned i = 0; i < sizeof(specialChars); ++i) {
        if(c == specialChars[i])
            return true;
    }

    return false;
}

int check_brac(char c) {
    if(c == '{') {
        return 1;
    }
    if(c == '}') {
        return -1;
    }
    return 0;
}
void skip(istream& is) {
    bool finishedReading = false;

    int check = 1;

    std::vector< string > tokens;
    while(!finishedReading) {
        check += read_line_as_tokens_chk(is, tokens);

        if(check == 0) {
            finishedReading = true;
        }
    }
}

int read_line_as_tokens_chk(istream& is, vector< string >& tokens) {
    tokens.clear();
    string line;
    std::getline(is, line);

    int chkBrac = 0;

    string token = "";

    for(unsigned i = 0; i < line.size(); ++i) {
        char currChar = line[i];
        bool isSpecialChar = is_special_char(currChar);
        chkBrac += check_brac(currChar);

        if(std::isspace(currChar) || isSpecialChar) {
            if(!token.empty()) {
                // Add the current token to the list of tokens
                tokens.push_back(token);
                token.clear();
            }
        }
        else {
            // Add the char to the current token
            token.push_back(currChar);
        }
    }

    if(!token.empty())
        tokens.push_back(token);

    return chkBrac;
}

bool read_line_as_tokens(istream& is, vector< string >& tokens,
                         bool includeSpecialChars) {
    tokens.clear();

    string line;
    std::getline(is, line);

    while(is && tokens.empty()) {
        string token = "";

        for(unsigned i = 0; i < line.size(); ++i) {
            char currChar = line[i];
            bool isSpecialChar = is_special_char(currChar);

            if(std::isspace(currChar) || isSpecialChar) {
                if(!token.empty()) {
                    // Add the current token to the list of tokens
                    tokens.push_back(token);
                    token.clear();
                }

                if(includeSpecialChars && isSpecialChar) {
                    tokens.push_back(string(1, currChar));
                }
            }
            else {
                // Add the char to the current token
                token.push_back(currChar);
            }
        }

        if(!token.empty())
            tokens.push_back(token);

        if(tokens.empty()) {
            // Previous line read was empty. Read the next one.
            std::getline(is, line);
        }
    }

    return !tokens.empty();
}

// Read the next line and return it as a list of tokens skipping white space and
// special characters
// The return value indicates success/failure.

bool Circuit::read_module(string& moduleName) {
    vector< string > tokens;
    bool valid = read_line_as_tokens(is, tokens);

    while(valid) {
        if(tokens.size() == 2 && tokens[0] == "module") {
            moduleName = tokens[1];
            break;
        }
        valid = read_line_as_tokens(is, tokens);
    }

    // Read and skip the port names in the module definition
    // until we encounter the tokens {"Start", "PIs"}
    while(valid &&
          !(tokens.size() == 2 && tokens[0] == "Start" && tokens[1] == "PIs")) {
        valid = read_line_as_tokens(is, tokens);
        assert(valid);
    }

    return valid;
}

bool Circuit::read_primary_input(string& primaryInput) {
    primaryInput = "";

    vector< string > tokens;
    bool valid = read_line_as_tokens(is, tokens);

    assert(valid);
    assert(tokens.size() == 2);

    if(valid && tokens[0] == "input") {
        primaryInput = tokens[1];
    }
    else {
        assert(tokens[0] == "Start" && tokens[1] == "POs");
        return false;
    }

    return valid;
}

bool Circuit::read_primary_output(string& primaryOutput) {
    primaryOutput = "";

    vector< string > tokens;
    bool valid = read_line_as_tokens(is, tokens);

    assert(valid);
    assert(tokens.size() == 2);

    if(valid && tokens[0] == "output") {
        primaryOutput = tokens[1];
    }
    else {
        assert(tokens[0] == "Start" && tokens[1] == "wires");
        return false;
    }

    return valid;
}

bool Circuit::read_wire(string& wire) {
    wire = "";

    vector< string > tokens;
    bool valid = read_line_as_tokens(is, tokens);

    assert(valid);
    assert(tokens.size() == 2);

    if(valid && tokens[0] == "wire") {
        wire = tokens[1];
    }
    else {
        assert(tokens[0] == "Start" && tokens[1] == "cells");
        return false;
    }

    return valid;
}

bool Circuit::read_cell_inst(
    string& cellType, string& cellInstName,
    vector< std::pair< string, string > >& pinNetPairs) {
    cellType = "";
    cellInstName = "";
    pinNetPairs.clear();

    vector< string > tokens;
    bool valid = read_line_as_tokens(is, tokens);

    assert(valid);

    if(tokens.size() == 1) {
        assert(tokens[0] == "endmodule");
        return false;
    }

    assert(
        tokens.size() >=
        4);  // We should have cellType, instName, and at least one pin-net pair

    cellType = tokens[0];
    cellInstName = tokens[1];

    for(unsigned i = 2; i < tokens.size() - 1; i += 2) {
        assert(tokens[i][0] == '.');  // pin names start with '.'
        string pinName =
            tokens[i].substr(1);  // skip the first character of tokens[i]

        pinNetPairs.push_back(std::make_pair(pinName, tokens[i + 1]));
    }

    return valid;
}

// Read clock definition
// Return value indicates if the last read was successful or not.
bool Circuit::read_clock(string& clockName, string& clockPort, double& period) {
    clockName = "";
    clockPort = "";
    period = 0.0;

    vector< string > tokens;
    bool valid = read_line_as_tokens(is, tokens);

    double uncertainty = 0.0;

    while(valid) {
        if(tokens.size() == 10 && tokens[0] == "create_clock" &&
           tokens[3] == "-name") {
            clockName = tokens[4];

            assert(tokens[5] == "-period");
            period = std::atof(tokens[6].c_str());
            assert(tokens[7] == "-waveform");
            // FIXME:

            assert(tokens[1] == "get_ports");
            clockPort = tokens[2];
        }

        if(tokens.size() == 5 && tokens[0] == "set_clock_uncertainty") {
            uncertainty = std::atof(tokens[2].c_str());
        }

        if(tokens.size() == 2 && tokens[0] == "input" &&
           tokens[1] == "delays") {
            // period = period - uncertainty;
        }
        if(tokens.size() == 3 && tokens[0] == "set_ideal_network" &&
           tokens[1] == "get_ports") {
            // period = period - uncertainty;
        }
        if(tokens.size() == 3 && tokens[0] == "set_input_transition" &&
           tokens[2] == "all_inputs") {
            // period = period - uncertainty;
        }
        valid = read_line_as_tokens(is, tokens);
    }

    // Skip the next comment line to prepare for the next stage
    // bool valid2 = read_line_as_tokens (is, tokens) ;
    // assert (valid2) ;
    // assert(tokens.size() == 2);
    // assert(tokens[0] == "input" && tokens[1] == "delays");

    return valid;
}

// Read input delay
// Return value indicates if the last read was successful or not.
bool Circuit::read_input_delay(string& portName, double& delay) {
    portName = "";
    delay = 0.0;

    vector< string > tokens;
    bool valid = read_line_as_tokens(is, tokens);

    assert(valid);
    assert(tokens.size() >= 2);

    if(valid && tokens[0] == "set_input_delay") {
        assert(tokens.size() == 6 || tokens.size() == 7);

        delay = std::atof(tokens[1].c_str());

        assert(tokens[2] == "get_ports");

        portName = tokens[3];

        if(tokens.size() == 6) {
            assert(tokens[4] == "-clock");
        }
        else {
            portName = portName + "[" + tokens[4] + "]";
            assert(tokens[5] == "-clock");
        }
    }
    else {
        assert(tokens.size() == 2);
        assert(tokens[0] == "input" && tokens[1] == "drivers");

        return false;
    }

    return valid;
}

// Read output delay
// Return value indicates if the last read was successful or not.
bool Circuit::read_output_delay(string& portName, double& delay) {
    portName = "";
    delay = 0.0;

    vector< string > tokens;
    bool valid = read_line_as_tokens(is, tokens);

    assert(valid);
    assert(tokens.size() >= 2);

    if(valid && tokens[0] == "set_output_delay") {
        assert(tokens.size() == 6 || tokens.size() == 7);

        delay = std::atof(tokens[1].c_str());

        assert(tokens[2] == "get_ports");

        portName = tokens[3];

        if(tokens.size() == 6) {
            assert(tokens[4] == "-clock");
        }
        else {
            portName = portName + "[" + tokens[4] + "]";
            assert(tokens[5] == "-clock");
        }
    }
    else {
        assert(tokens.size() == 2);
        assert(tokens[0] == "output" && tokens[1] == "loads");
        return false;
    }

    return valid;
}

// Read driver info for the input port
// Return value indicates if the last read was successful or not.
bool Circuit::read_driver_info(string& inPortName, string& driverSize,
                               string& driverPin, double& inputTransitionFall,
                               double& inputTransitionRise) {
    inPortName = "";
    driverSize = "";
    driverPin = "";
    inputTransitionFall = 0.0;
    inputTransitionRise = 0.0;

    vector< string > tokens;
    bool valid = read_line_as_tokens(is, tokens);

    assert(valid);
    assert(tokens.size() >= 2);

    if(valid && tokens[0] == "set_driving_cell") {
        assert(tokens.size() == 11 || tokens.size() == 12);
        assert(tokens[1] == "-lib_cell");

        driverSize = tokens[2];

        assert(tokens[3] == "-pin");
        driverPin = tokens[4];

        assert(tokens[5] == "get_ports");
        inPortName = tokens[6];

        if(tokens.size() == 11) {
            assert(tokens[7] == "-input_transition_fall");
            inputTransitionFall = std::atof(tokens[8].c_str());

            assert(tokens[9] == "-input_transition_rise");
            inputTransitionRise = std::atof(tokens[10].c_str());
        }
        else {
            inPortName = inPortName + "[" + tokens[7] + "]";
            assert(tokens[8] == "-input_transition_fall");
            inputTransitionFall = std::atof(tokens[9].c_str());

            assert(tokens[10] == "-input_transition_rise");
            inputTransitionRise = std::atof(tokens[11].c_str());
        }
    }
    else {
        assert(tokens.size() == 2);
        assert(tokens[0] == "output" && tokens[1] == "delays");

        return false;
    }

    return valid;
}

// Read output load
// Return value indicates if the last read was successful or not.
bool Circuit::read_output_load(string& outPortName, double& load) {
    outPortName = "";
    load = 0.0;

    vector< string > tokens;
    bool valid = read_line_as_tokens(is, tokens);

    if(valid && tokens[0] == "set_load") {
        assert(tokens.size() == 5 || tokens.size() == 6);

        assert(tokens[1] == "-pin_load");
        load = std::atof(tokens[2].c_str());

        assert(tokens[3] == "get_ports");
        outPortName = tokens[4];
        if(tokens.size() == 6) {
            outPortName = outPortName + "[" + tokens[5] + "]";
        }
    }
    else {
        assert(!valid);
        return false;
    }

    return valid;
}

// Read timing info for the next pin
// Return value indicates if the last read was successful or not.
bool Circuit::read_pin_timing(string& cellInst, string& pin, double& riseSlack,
                              double& fallSlack, double& riseTransition,
                              double& fallTransition) {
    vector< string > tokens;
    bool valid = read_line_as_tokens(is, tokens);

    assert(valid);
    assert(tokens.size() >= 2);

    if(tokens[0] == "port" && tokens[1] == "timing") {
        // The next section (port timing) will start after this point
        tokens.clear();
        return false;
    }
    else {
        if(tokens.size() != 6) {
            // cout << "token size : "<< tokens.size() <<endl;
            // for(unsigned i=0 ; i<tokens.size() ; i++)
            //    cout << tokens[i] << " ";
        }

        assert(tokens.size() == 6);
        cellInst = tokens[0];
        pin = tokens[1];
        riseSlack = std::atof(tokens[2].c_str());
        fallSlack = std::atof(tokens[3].c_str());
        riseTransition = std::atof(tokens[4].c_str());
        fallTransition = std::atof(tokens[5].c_str());
        tokens.clear();

        return true;
    }
}

// Read timing info for the next port
// Return value indicates if the last read was successful or not.
bool Circuit::read_port_timing(string& port, double& riseSlack,
                               double& fallSlack, double& riseTransition,
                               double& fallTransition) {
    vector< string > tokens;
    bool valid = read_line_as_tokens(is, tokens);

    if(valid) {
        assert(tokens.size() == 5);
        port = tokens[0];
        riseSlack = std::atof(tokens[1].c_str());
        fallSlack = std::atof(tokens[2].c_str());
        riseTransition = std::atof(tokens[3].c_str());
        fallTransition = std::atof(tokens[4].c_str());
    }

    return valid;
}

// No need to parse the 3D LUTs, because they will be ignored
void Circuit::_skip_lut_3D() {
    std::vector< string > tokens;

    bool valid = read_line_as_tokens(is, tokens);
    assert(valid);
    assert(tokens[0] == "index_1");
    assert(tokens.size() >= 2);
    int size1 = tokens.size() - 1;

    valid = read_line_as_tokens(is, tokens);
    assert(valid);
    assert(tokens[0] == "index_2");
    assert(tokens.size() >= 2);
    int size2 = tokens.size() - 1;

    valid = read_line_as_tokens(is, tokens);
    assert(valid);
    assert(tokens[0] == "index_3");
    assert(tokens.size() >= 2);
    int size3 = tokens.size() - 1;

    valid = read_line_as_tokens(is, tokens);
    assert(valid);
    assert(tokens.size() == 1 && tokens[0] == "values");

    for(int i = 0; i < size1; ++i) {
        for(int j = 0; j < size2; ++j) {
            valid = read_line_as_tokens(is, tokens);
            assert(valid);
            assert(tokens.size() == (unsigned)size3);
        }
    }
}

string Circuit::_begin_read_timing_info(istream& is, string toPin,
                                        LibTimingInfo& timing, LibInfo lib) {
    timing.toPin = toPin;

    bool finishedReading = false;

    unsigned timingType = 0;

    int check = 1;
    std::vector< string > tokens;

    while(!finishedReading) {
        check += read_line_as_tokens_chk(is, tokens);

        if(check == 0) {
            finishedReading = true;
        }

        if(tokens.size() == 0) {
            continue;
        }

        if(tokens[0] == "cell_fall" ||
           (timingType == 1 && tokens[0] == "fall_constraint")) {
            if(tokens[1] == "scalar") {
                skip(is);
                --check;
            }
            else {
                timing.fallDelay.templ = tokens[1];
                // cout << "read fall delay table" << endl;
                _begin_read_lut(is, timing.fallDelay, "timing", lib);
            }
        }
        else if(tokens[0] == "cell_rise" ||
                (timingType == 1 && tokens[0] == "rise_constraint")) {
            if(tokens[1] == "scalar") {
                skip(is);
                --check;
            }
            else {
                // cout << "read rise delay table" << endl;
                timing.riseDelay.templ = tokens[1];
                _begin_read_lut(is, timing.riseDelay, "timing", lib);
            }
        }
        else if(tokens[0] == "fall_transition" ||
                (timingType == 2 && tokens[0] == "fall_constraint")) {
            if(tokens[1] == "scalar") {
                skip(is);
                --check;
            }
            else {
                timing.fallTransition.templ = tokens[1];
                _begin_read_lut(is, timing.fallTransition, "timing", lib);
            }
        }
        else if(tokens[0] == "rise_transition" ||
                (timingType == 2 && tokens[0] == "rise_constraint")) {
            if(tokens[1] == "scalar") {
                skip(is);
                --check;
            }
            else {
                timing.riseTransition.templ = tokens[1];
                _begin_read_lut(is, timing.riseTransition, "timing", lib);
            }
        }
        else if(tokens[0] == "timing_sense") {
            if(tokens[1] == "positive_unate") {
                timing.timingSense = 'p';
            }
            else if(tokens[1] == "negative_unate") {
                timing.timingSense = 'n';
            }
            else {
                timing.timingSense = 'p';
            }
        }
        else if(tokens[0] == "related_pin") {
            assert(tokens.size() == 2);
            timing.fromPin = tokens[1];
        }
        else if(tokens[0] == "timing_type") {
            if(tokens[1].find("setup") != -1) {
                // cout << "setup timing" << endl;
                timing.timingSense = 'c';
                timingType = 1;
            }
            else if(tokens[1].find("hold") != -1) {
                timingType = 2;
                timing.timingSense = 'c';
                // cout << "hold timing" << endl;
            }
        }
        else if(tokens[0] == "when") {
            timing.isFunc = true;
        }
    }
    return timing.fromPin;
}

void Circuit::_begin_read_pin_info(istream& is, string pinName, LibPinInfo& pin,
                                   LibCellInfo& cell, LibInfo& lib) {
    // cout << "Reading pin " << pinName << endl;

    pin.name = pinName;
    pin.isClock = false;
    pin.isData = false;

    bool finishedReading = false;

    int check = 1;
    std::vector< string > tokens;
    while(!finishedReading) {
        check += read_line_as_tokens_chk(is, tokens);
        if(check == 0) {
            finishedReading = true;
        }

        if(tokens.size() == 0) {
            continue;
        }

        if(tokens.size() == 2 && tokens[0] == "direction") {
            if(tokens[1] == "input") {
                pin.isInput = true;
                pin.isOutput = false;
            }
            else if(tokens[1] == "output") {
                pin.isInput = false;
                pin.isOutput = true;
            }
            else if(tokens[1] == "inout") {
                pin.isInput = true;
                pin.isOutput = true;
            }
            else if(tokens[1] == "internal") {
                pin.isInput = false;
                pin.isOutput = false;
            }
            else {
                assert(false);  // undefined direction
            }
        }
        else if(tokens.size() == 2 && tokens[0] == "capacitance") {
            pin.capacitance = std::atof(tokens[1].c_str());
        }
        else if(tokens.size() == 2 && tokens[0] == "max_capacitance") {
            pin.maxCapacitance = std::atof(tokens[1].c_str());
        }
        else if(tokens[0] == "internal_power" && pin.isOutput && !pin.isInput) {
            LibPowerInfo tmplib;

            tmplib.isFunc = false;
            tmplib.cnt = 1;

            unsigned relatedPinId, curPinId;
            string relatedPin =
                _begin_read_power_info(is, pinName, tmplib, lib);
            if(relatedPin != "") {
                // relatedPin Id
                if(cell.lib_pin2id_map.find(relatedPin) ==
                   cell.lib_pin2id_map.end()) {
                    relatedPinId = cell.lib_pin2id_map.size();
                    // cell.lib_pin2id_map[relatedPin] = relatedPinId;
                    cell.lib_pin2id_map.insert(
                        pair< string, unsigned >(relatedPin, relatedPinId));
                    // cout << "ADD PIN " << cell.name << "/" << relatedPin << "
                    // " << cell.lib_pin2id_map[relatedPin] << endl;
                }
                else {
                    relatedPinId = cell.lib_pin2id_map[relatedPin];
                }

                // curPin Id
                if(cell.lib_pin2id_map.find(pinName) ==
                   cell.lib_pin2id_map.end()) {
                    curPinId = cell.lib_pin2id_map.size();
                    // cell.lib_pin2id_map[pinName] = curPinId;
                    cell.lib_pin2id_map.insert(
                        pair< string, unsigned >(pinName, curPinId));
                    // cout << "ADD PIN " << cell.name << "/" << pinName << " "
                    // << cell.lib_pin2id_map[pinName] << endl;
                }
                else {
                    curPinId = cell.lib_pin2id_map[pinName];
                }
                // JLPWR

                if(relatedPinId != curPinId) {
                    // unsigned index = relatedPinId*100 + curPinId;
                    unsigned index = relatedPinId + curPinId * 100;
                    if(cell.powerTables.find(index) != cell.powerTables.end()) {
                        merge_powerTables(cell.powerTables[index], tmplib);
                    }
                    else {
                        cell.powerTables.insert(
                            pair< unsigned, LibPowerInfo >(index, tmplib));
                    }
                }

                // cout << "Read " << cell.powerTables.size() << " power tables"
                // << endl;
            }
            --check;
        }
        else if(tokens[0] == "timing") {
            LibTimingInfo tmplib;
            string fromPin;
            unsigned fromPinId, toPinId;

            tmplib.isFunc = false;
            tmplib.cnt0 = 1;
            tmplib.cnt1 = 1;
            tmplib.cnt2 = 1;
            tmplib.cnt3 = 1;

            // Read timing info
            fromPin = _begin_read_timing_info(is, pinName, tmplib, lib);
            // cout << "fromPin: " << fromPin << " toPin: " << pinName << endl;

            if(fromPin != "") {
                // JLPWR
                if(tmplib.timingSense == '-') {
                    if(pin.IQN)
                        tmplib.timingSense = 'n';
                    else
                        tmplib.timingSense = 'p';
                }

                if(tmplib.timingSense == 'c') {
                    pin.isData = true;
                }

                // fromPin Id
                if(cell.lib_pin2id_map.find(fromPin) ==
                   cell.lib_pin2id_map.end()) {
                    fromPinId = cell.lib_pin2id_map.size();
                    // cell.lib_pin2id_map[fromPin] = fromPinId;
                    cell.lib_pin2id_map.insert(
                        pair< string, unsigned >(fromPin, fromPinId));
                    // cout << "ADD PIN " << cell.name << "/" << fromPin << " "
                    // << cell.lib_pin2id_map[fromPin] << endl;
                }
                else {
                    fromPinId = cell.lib_pin2id_map[fromPin];
                }

                // toPin Id
                if(cell.lib_pin2id_map.find(pinName) ==
                   cell.lib_pin2id_map.end()) {
                    toPinId = cell.lib_pin2id_map.size();
                    // cell.lib_pin2id_map[pinName] = toPinId;
                    cell.lib_pin2id_map.insert(
                        pair< string, unsigned >(pinName, toPinId));
                    // cout << "ADD PIN " << cell.name << "/" << pinName << " "
                    // << cell.lib_pin2id_map[pinName] << endl;
                }
                else {
                    toPinId = cell.lib_pin2id_map[pinName];
                }
                if(fromPinId != toPinId) {
                    unsigned index = fromPinId + toPinId * 100;
                    // Update existing timing info
                    if(cell.timingArcs.find(index) != cell.timingArcs.end()) {
                        merge_timingArcs(cell.timingArcs[index], tmplib);
                    }
                    else {
                        if(!tmplib.riseDelay.tableVals.empty() ||
                           !tmplib.fallDelay.tableVals.empty() ||
                           !tmplib.riseTransition.tableVals.empty() ||
                           !tmplib.fallTransition.tableVals.empty()) {
                            cell.timingArcs.insert(
                                pair< unsigned, LibTimingInfo >(index, tmplib));
                        }
                    }
                }
                // cout << "Read " << cell.timingArcs.size() << " timing arcs"
                // << endl;
            }
            --check;
        }
        else if(tokens[0] == "clock") {
            pin.isClock = true;
            cell.isSequential = true;
        }
        else if(tokens[0] == "clock_gate_clock_pin" && tokens[1] == "true") {
            pin.isClock = true;
            cell.isSequential = true;
        }
        else if(tokens[0] == "nextstate_type" && tokens[1] == "data") {
            pin.isData = true;

            // add EN pins
        }
        else if(tokens[0] == "clock_gate_enable_pin" && tokens[1] == "true") {
            pin.isData = true;
        }
        else if(tokens[0] == "function" && tokens[1] == "IQN") {
            // cout << "Function == IQN" << endl;
            cell.hasQN = true;
            pin.IQN = true;
        }
    }
}

void Circuit::_begin_read_cell_info(istream& is, LibCellInfo& cell,
                                    LibInfo& lib) {
    cell.isSequential = false;
    cell.dontTouch = false;

    bool finishedReading = false;
    int check = 1;

    double leak = 0;
    unsigned leak_cnt = 0;
    // true: one leakage value / false: average over state-dependent values
    bool leak_flag = false;
    string data_pin = "";

    std::vector< string > tokens;
    while(!finishedReading) {
        check += read_line_as_tokens_chk(is, tokens);
        if(check == 0) {
            finishedReading = true;
            // assign leakage power
            if(!leak_flag) {
                if(leak_cnt != 0 && lib.leak_power_unit != 0)
                    // cell.leakagePower = leak / leak_cnt * lib.leak_power_unit
                    // / 1e-3;
                    cell.leakagePower = leak / leak_cnt;
                else
                    cell.leakagePower = 0.0;
            }
            if(cell.footprint == "" || NO_FOOTPRINT) {
                cell.footprint = cell.name;
                if(VERBOSE > 1)
                    cout << "GET FOOTPRINT FOR " << cell.footprint << endl;
                size_t start = cell.footprint.find_first_of("_");
                if(start != -1) {
                    cell.footprint.erase(start,
                                         cell.footprint.length() - start);
                }
                else
                    cell.footprint = "NA";
                if(ASAP7) {
                    string temp = cell.name;
                    size_t start = temp.find_first_of("x");
                    temp.erase(start, temp.length() - start);
                    cell.footprint = temp;
                    // cout << "footprint " << cell.name << " " <<
                    //         cell.footprint << endl;
                }
                if(STM28) {
                    string temp = cell.name;
                    string delimiter = "_";
                    size_t pos = temp.find(delimiter);
                    size_t pos1 = temp.find(delimiter, pos + 1);
                    size_t pos2 = temp.find(delimiter, pos1 + 1);
                    temp = temp.substr(pos1 + 1, pos2);

                    size_t start = temp.find_last_of("X");
                    temp.erase(start, temp.length() - start);
                    cell.footprint = temp;
                    // cout << "footprint " << cell.name << " " <<
                    // cell.footprint << endl;
                }
                if(C40) {
                    string temp = cell.name;
                    string delimiter = "_";
                    size_t pos = temp.find(delimiter);
                    size_t pos1 = temp.find(delimiter, pos + 1);
                    temp = temp.substr(pos + 1, pos1);
                    cell.footprint = temp;
                    // cout << "footprint " << cell.name << " " <<
                    // cell.footprint << endl;
                }
            }
            // cout << "Finish reading cell " << cell.name << endl;
        }

        if(tokens.size() == 0) {
            continue;
        }

        if(tokens.size() == 1 && tokens[0] == "test_cell") {
            skip(is);
            check--;
        }

        if(tokens.size() == 2 && tokens[0] == "cell_leakage_power") {
            // Normalize the leakage power to mW
            // cell.leakagePower = atof(tokens[1].c_str()) * lib.leak_power_unit
            // / 1e-3;
            cell.leakagePower = atof(tokens[1].c_str());
            leak_flag = true;
        }
        else if(tokens.size() == 1 && tokens[0] == "leakage_power") {
            read_leak(is, leak, leak_cnt);
            --check;
        }
        else if(tokens[0] == "cell_footprint") {
            assert(tokens.size() == 2);
            cell.footprint = tokens[1];
        }
        else if(tokens[0] == "area") {
            assert(tokens.size() == 2);
            cell.area = std::atof(tokens[1].c_str());
            // Missing cell height info (specify cell.width = cell.area)
            cell.width = std::atof(tokens[1].c_str());
        }
        else if(tokens[0] == "clocked_on") {
            cell.isSequential = true;
        }
        else if(tokens[0] == "dont_touch") {
            cell.dontTouch = true;
        }
        else if(tokens.size() == 2 && tokens[0] == "next_state") {
            data_pin = tokens[1];
        }
        else if(tokens.size() == 2 && tokens[0] == "pin") {
            LibPinInfo pin;
            _begin_read_pin_info(is, tokens[1], pin, cell, lib);

            if(cell.lib_pin2id_map.find(pin.name) ==
               cell.lib_pin2id_map.end()) {
                unsigned pin_id = cell.lib_pin2id_map.size();
                cell.lib_pin2id_map.insert(
                    pair< string, unsigned >(pin.name, pin_id));
                // cout << "ADD PIN " << cell.name << "/" << pin.name << " " <<
                // cell.lib_pin2id_map[pin.name] << endl;
            }
            cell.pins.insert(pair< unsigned, LibPinInfo >(
                cell.lib_pin2id_map[pin.name], pin));
            --check;
        }
        else if(tokens.size() == 2 && tokens[0] == "dont_use" &&
                tokens[1] == "true") {
            cell.dontUse = true;
        }
    }
    std::map< unsigned, LibTimingInfo >::iterator timeItr;
    for(timeItr = cell.timingArcs.begin(); timeItr != cell.timingArcs.end();
        ++timeItr) {
        if(timeItr->second.cnt0 + timeItr->second.cnt1 + timeItr->second.cnt2 +
               timeItr->second.cnt3 >
           4) {
            average_lut(timeItr->second.fallDelay, timeItr->second.cnt0);
            average_lut(timeItr->second.riseDelay, timeItr->second.cnt1);
            average_lut(timeItr->second.fallTransition, timeItr->second.cnt2);
            average_lut(timeItr->second.riseTransition, timeItr->second.cnt3);
            timeItr->second.cnt0 = 1;
            timeItr->second.cnt1 = 1;
            timeItr->second.cnt2 = 1;
            timeItr->second.cnt3 = 1;
        }
    }
    std::map< unsigned, LibPowerInfo >::iterator powerItr;
    for(powerItr = cell.powerTables.begin(); powerItr != cell.powerTables.end();
        ++powerItr) {
        if(powerItr->second.cnt > 1) {
            int cnt = powerItr->second.cnt;
            if(powerItr->second.pgPins.size() != 0) {
                cnt = cnt / powerItr->second.pgPins.size();
            }
            if(powerItr->second.fallPower.templ != "") {
                average_lut(powerItr->second.fallPower, cnt);
            }
            if(powerItr->second.risePower.templ != "") {
                average_lut(powerItr->second.risePower, cnt);
            }
            powerItr->second.cnt = 1;
        }
    }
    if(VERBOSE > 4)
        cout << cell;

    std::map< unsigned, LibPinInfo >::iterator pinItr;
    for(pinItr = cell.pins.begin(); pinItr != cell.pins.end(); ++pinItr) {
        if(pinItr->second.isInput && pinItr->second.name == data_pin) {
            pinItr->second.isData = 1;
        }
    }
}

// Read the default max_transition defined for the library.
// Return value indicates if the last read was successful or not.
// This function must be called in the beginning before any read_cell_info
// function call.
bool Circuit::read_default_max_transition(double& maxTransition) {
    maxTransition = 0.0;
    vector< string > tokens;

    // bool valid = read_line_as_tokens (is, tokens) ;
    read_line_as_tokens(is, tokens);

    // while (valid) {
    while(!is.eof()) {
        if(tokens.size() == 2 && tokens[0] == "default_max_transition") {
            maxTransition = std::atof(tokens[1].c_str());
            cout << "MAX tran " << maxTransition << endl;
            return true;
        }

        // valid = read_line_as_tokens (is, tokens) ;
        read_line_as_tokens(is, tokens);
    }

    return false;
}

void Circuit::read_leak(istream& is, double& leak, unsigned& leak_cnt) {
    unsigned check = 1;

    vector< string > tokens;

    while(check != 0) {
        check += read_line_as_tokens_chk(is, tokens);
        if(tokens.size() == 2 && tokens[0] == "value") {
            leak += atof(tokens[1].c_str());
            ++leak_cnt;
        }
    }
}

ostream& operator<<(ostream& os, LibPowerInfo& power) {
    cout << "Power related to " << power.relatedPin << " to " << power.toPin
         << ": " << endl;

    cout << "Fall power LUT: " << endl;
    cout << power.fallPower;

    cout << "Rise power LUT: " << endl;
    cout << power.risePower;

    return os;
}

ostream& operator<<(ostream& os, LibLUT& lut) {
    if(lut.loadIndices.empty() && lut.transitionIndices.empty() &&
       lut.tableVals.empty())
        return os;

    cout << "template: " << lut.templ << endl;

    cout << "\t";
    for(unsigned i = 0; i < lut.transitionIndices.size(); ++i) {
        cout << lut.transitionIndices[i] << "\t";
    }
    cout << endl;

    for(unsigned i = 0; i < lut.loadIndices.size(); ++i) {
        cout << lut.loadIndices[i] << "\t";

        for(unsigned j = 0; j < lut.transitionIndices.size(); ++j)
            cout << lut.tableVals[i][j] << "\t";

        cout << endl;
    }

    return os;
}

ostream& operator<<(ostream& os, LibTimingInfo& timing) {
    cout << "Timing info from " << timing.fromPin << " to " << timing.toPin
         << ": " << endl;
    cout << "Timing sense: " << timing.timingSense << endl;

    cout << "Fall delay LUT: " << endl;
    cout << timing.fallDelay;

    cout << "Rise delay LUT: " << endl;
    cout << timing.riseDelay;

    cout << "Fall transition LUT: " << endl;
    cout << timing.fallTransition;

    cout << "Rise transition LUT: " << endl;
    cout << timing.riseTransition;

    return os;
}

ostream& operator<<(ostream& os, LibPinInfo& pin) {
    cout << "Pin " << pin.name << ":" << endl;
    // cout << "PinID: " << pin.lib_pin_id << endl ;
    cout << "capacitance: " << pin.capacitance << endl;
    cout << "maxCapacitance: " << pin.maxCapacitance << endl;
    cout << "isInput? " << (pin.isInput ? "true" : "false") << endl;
    cout << "isClock? " << (pin.isClock ? "true" : "false") << endl;
    cout << "isData? " << (pin.isData ? "true" : "false") << endl;
    cout << "End pin" << endl;

    return os;
}

ostream& operator<<(ostream& os, LibCellInfo& cell) {
    cout << "Library cell " << cell.name << ": " << endl;

    cout << "Footprint: " << cell.footprint << endl;
    cout << "Leakage power: " << cell.leakagePower << endl;
    cout << "Area: " << cell.area << endl;
    cout << "Sequential? " << (cell.isSequential ? "yes" : "no") << endl;
    cout << "Dont-touch? " << (cell.dontTouch ? "yes" : "no") << endl;

    cout << "Cell has " << cell.pins.size() << " pins: " << endl;
    std::map< unsigned, LibPinInfo >::iterator it1;
    for(it1 = cell.pins.begin(); it1 != cell.pins.end(); ++it1) {
        cout << it1->second << endl;
    }
    cout << "Cell has " << cell.timingArcs.size() << " timing arcs: " << endl;
    std::map< unsigned, LibTimingInfo >::iterator it2;
    for(it2 = cell.timingArcs.begin(); it2 != cell.timingArcs.end(); ++it2) {
        cout << it2->second << endl;
    }
    cout << "Cell has " << cell.powerTables.size() << " power tables: " << endl;
    std::map< unsigned, LibPowerInfo >::iterator it3;
    for(it3 = cell.powerTables.begin(); it3 != cell.powerTables.end(); ++it3) {
        cout << it3->second << endl;
    }

    cout << "End of cell " << cell.name << endl << endl;

    return os;
}

ostream& operator<<(ostream& os, SOL& sol) {
    cout << "Cost: Cell " << sol.cell_name;
    cout << "/Inst " << sol.inst_name;
    cout << "/x " << sol.x;
    cout << "/width " << sol.width;
    cout << "/vt " << sol.vt;
    cout << "/delta x " << sol.delta_x;
    cout << "/best leak " << sol.cost_leak;
    cout << "/best move " << sol.cost_move;
    cout << "/best leak prev" << sol.best_leak_prev;
    cout << "/best move prev" << sol.best_move_prev << endl;
    return os;
}

void Circuit::merge_timingArcs(LibTimingInfo& timing1, LibTimingInfo& timing2) {
    if(timing1.isFunc && !timing2.isFunc) {
        if(timing2.fallDelay.templ == "") {
            timing1.fallDelay.tableVals.clear();
        }
        else {
            timing1.fallDelay = timing2.fallDelay;
        }
        if(timing2.riseDelay.templ == "") {
            timing1.riseDelay.tableVals.clear();
        }
        else {
            timing1.riseDelay = timing2.riseDelay;
        }
        if(timing2.fallTransition.templ == "") {
            timing1.fallTransition.tableVals.clear();
        }
        else {
            timing1.fallTransition = timing2.fallTransition;
        }
        if(timing2.riseTransition.templ == "") {
            timing1.riseTransition.tableVals.clear();
        }
        else {
            timing1.riseTransition = timing2.riseTransition;
        }
        timing1.isFunc = false;
        timing1.cnt0 = 1;
        timing1.cnt1 = 1;
        timing1.cnt2 = 1;
        timing1.cnt3 = 1;
    }
    else if(timing1.isFunc == timing2.isFunc) {
        if(timing2.fallDelay.templ != "") {
            if(timing1.fallDelay.templ != "") {
                ++timing1.cnt0;
            }
            update_lut(timing1.fallDelay, timing2.fallDelay);
        }
        if(timing2.riseDelay.templ != "") {
            if(timing1.riseDelay.templ != "") {
                ++timing1.cnt1;
            }
            update_lut(timing1.riseDelay, timing2.riseDelay);
        }
        if(timing2.fallTransition.templ != "") {
            if(timing1.fallTransition.templ != "") {
                ++timing1.cnt2;
            }
            update_lut(timing1.fallTransition, timing2.fallTransition);
        }
        if(timing2.riseTransition.templ != "") {
            if(timing1.riseTransition.templ != "") {
                ++timing1.cnt3;
            }
            update_lut(timing1.riseTransition, timing2.riseTransition);
        }
    }
}

void Circuit::merge_powerTables(LibPowerInfo& power1, LibPowerInfo& power2) {
    if(power1.isFunc && !power2.isFunc) {
        if(power2.fallPower.templ != "") {
            power1.fallPower = power2.fallPower;
        }
        if(power2.risePower.templ != "") {
            power1.risePower = power2.risePower;
        }
        power1.isFunc = false;
        power1.cnt = 1;
    }
    else if(!power1.isFunc && !power2.isFunc) {
        bool flag = false;
        for(unsigned i = 0; i < power1.pgPins.size(); ++i) {
            if(power1.pgPins[i] == power2.pgPins[0]) {
                flag = true;
                break;
            }
        }
        if(!flag) {
            if(power2.fallPower.templ != "") {
                update_lut(power1.fallPower, power2.fallPower);
            }
            if(power2.risePower.templ != "") {
                update_lut(power1.risePower, power2.risePower);
            }
            add_pg_pin(power1, power2.pgPins[0]);
        }
    }
    else if(power1.isFunc && power2.isFunc) {
        if(power2.fallPower.templ != "") {
            update_lut(power1.fallPower, power2.fallPower);
        }
        if(power2.risePower.templ != "") {
            update_lut(power1.risePower, power2.risePower);
        }
        add_pg_pin(power1, power2.pgPins[0]);
        ++power1.cnt;
    }
}

void Circuit::init_opensta(sta::Sta* _sta) {
    string netlistFileName = _sizer->verilogFile;
    string sdcFileName = _sizer->sdcFile;
    string spefFileName = _sizer->spefFile;
    vector< string > libs = _sizer->libLibs;
    string libPath = _sizer->libLibPath;

    string top_cell_name = _sizer->benchname;
    string clk_name = _sizer->clockName;

    // vector <LibertyLibrary*> libLibs;

    StringSet corner_names;
    corner_names.insert("0");
    string cornerName = "0";

    // Tcl Interpreter settings
#if 0
    sta_interp = Tcl_CreateInterp();
    Tcl_Init(sta_interp);
    //   // Define swig commands.
    // sta_swig_init(sta_interp);

    // define swig commands
    Sta_Init(sta_interp);
    initSta();
    sta::Sta::setSta(_sta);
    _sta->makeComponents();
    _sta->setTclInterp(sta_interp);

    // load encoded TCL functions
    evalTclInitForLibrary(sta_interp, tcl_inits);

    // initialize TCL commands
    Tcl_Eval(sta_interp, "sta::show_splash");
    Tcl_Eval(sta_interp, "sta::define_sta_cmds");
    Tcl_Eval(sta_interp, "namespace import sta::*");
    // string sta_init_str = "source -verbose [file join $env(HOME) .sta]";
    // Tcl_Eval(sta_interp, sta_init_str.c_str());
    // std::cout << Tcl_GetStringResult(sta_interp) << std::endl;
    // initialize

    cout << "STA CREATED" << endl;

    // define_corners
    _sta->makeCorners(&corner_names);
    Corner* corner = _sta->findCorner(cornerName.c_str());
    cout << "define_corner done" << endl;

    for(int i = 0; i < libs.size(); i++) {
        string lib_file_name = libPath + "/" + libs[i].c_str();
        cout << "LIB FILE NAME : " << lib_file_name << endl;
        LibertyLibrary* lib_tmp = _sta->readLiberty(
            lib_file_name.c_str(), corner, MinMaxAll::max(), false);
        // libLibs.push_back(lib_tmp);
    }
    cout << "read_liberty done" << endl;

    // read_netlist
    Network* network = _sta->networkReader();
    bool readVerilog = false;
    if(network) {
        _sta->readNetlistBefore();
        readVerilog = sta::readVerilogFile(netlistFileName.c_str(), _sta->networkReader());
    }
    cout << "read_verilog done : " << readVerilog << endl;

    // link_design
    bool link = _sta->linkDesign(top_cell_name.c_str());
    cout << "link_design done : " << link << endl;
    bool is_linked = network->isLinked();
    if(is_linked)
        cout << "linked: " << network->cellName(_sta->currentInstance())
             << endl;

    // read_parasitics
    bool parasitics = _sta->readSpef(
        spefFileName.c_str(), _sta->currentInstance(), MinMaxAll::max(),
        true, false, 0.0, true);
    cout << "read_parasitics done : " << parasitics << endl;

    string sdc_cmd = "read_sdc " + sdcFileName;
    std::cout << std::string(evalTclString(sdc_cmd)) << std::endl;
    std::cout << "read_sdc done !!!" << std::endl;
    // string setrc_cmd = "source " + libPath + "/../setRC.tcl";
    // std::cout << std::string(evalTclString(setrc_cmd)) << std::endl;
#endif
}

// 最慢
void Circuit::readDesign_opensta(sta::Sta* _sta) {
    string netlistFileName = _sizer->verilogFile;
    string sdcFileName = _sizer->sdcFile;
    string spefFileName = _sizer->spefFile;
    vector< string > libs = _sizer->libLibs;
    string top_cell_name = _sizer->benchname;
    string clk_name = _sizer->clockName;

    Network* network = _sta->networkReader();

    string strLibName;
    string strCellName;
    string strViewName;

    strLibName = _sizer->benchname;
    strCellName = _sizer->benchname;
    strViewName = "abstract";

    string libName = _sizer->benchname;
    string cellName = _sizer->benchname;
    string viewName = strViewName;
    string libPath = _sizer->benchname;

    int gateNum = network->instanceCount();

    // read in the gates
    InstanceChildIterator* inst_it =
        network->childIterator(network->topInstance());
    int iter_i = 0;
    while(inst_it->hasNext()) {
        Instance* inst = inst_it->next();
        if(iter_i % 1000 == 0) {
            printf("Read %d / %d Instances\n", iter_i, gateNum);
        }
        iter_i++;
        string strCellName = network->libertyCell(inst)->name();

        // NEW CELL
        CELL tmpCell;
        tmpCell.type = network->libertyCell(inst)->name();
        tmpCell.name = network->pathName(inst);
        tmpCell.isFF = false;

        if(_sizer->numVt == 3) {
            if(cellName.find(_sizer->suffixLVT.c_str()) != std::string::npos) {
                tmpCell.c_vtype = f;
                // tmpCell.orig_c_vtype = f;
            }
            else if(cellName.find(_sizer->suffixHVT.c_str()) !=
                    std::string::npos) {
                tmpCell.c_vtype = s;
                // tmpCell.orig_c_vtype = s;
            }
            else {
                tmpCell.c_vtype = m;
                // tmpCell.orig_c_vtype = m;
            }
        }
        else if(_sizer->numVt == 2) {
            if(cellName.find(_sizer->suffixHVT.c_str()) != std::string::npos) {
                tmpCell.c_vtype = s;
                // tmpCell.orig_c_vtype = s;
            }
            else {
                tmpCell.c_vtype = m;
                // tmpCell.orig_c_vtype = m;
            }
        }
        else {
            tmpCell.c_vtype = s;
            // tmpCell.orig_c_vtype = s;
        }

        unsigned tmpCellId = _sizer->_ckt->g_cells.size();

        _sizer->_ckt->cell2id.insert(
            pair< string, unsigned >(network->pathName(inst), tmpCellId));

        //       cout << "Name and ID: " << network->pathName(inst) << " : " <<
        //       tmpCellId << endl;

        _sizer->_ckt->g_cells.push_back(tmpCell);
    }

    // NET ITERATION
    // 最慢
    string pattern_str = "*";
    Instance* top_inst = network->topInstance();
    NetSeq* nets = new NetSeq;
    PatternMatch pattern(pattern_str.c_str(),
                         /* is_regexp */ true,
                         /* nocase */ true,
                         /* Tcl_interp* */ nullptr);
    network->findNetsHierMatching(top_inst, &pattern, *nets);
    int netsNum = nets->size();
    NetSeq::Iterator nets_iter(nets);
    iter_i = 0;
    while(nets_iter.hasNext()) {
        auto net = nets_iter.next();
        if(iter_i % 1000 == 0) {
            printf("Read %d / %d Nets\n", iter_i, netsNum);
        }
        iter_i++;

        string netName = network->pathName(net);
        if(netName.substr(0, 11) == "UNCONNECTED") {
            // con("debug debug!!");
            continue;
        }
        if(strcmp(netName.c_str(), string("SE").c_str()) == 0 ||
           strcmp(netName.c_str(), string("SI").c_str()) == 0 ||
           strcmp(netName.c_str(), string("SO").c_str()) == 0 ||
           strcmp(netName.c_str(), string("HRESETn").c_str()) == 0)
            continue;
        // cout << "Net Name: " << network->pathName(net) << endl;

        if(network->isPower(net) || network->isGround(net)) {
            // logFile << "power / ground net " << endl;
            continue;
        }

        // if(network->netCount() == 0) {
        //     // logFile << "0 term net: " << netName <<" found : ignore."<<
        //     endl; continue;
        // }

        // NEW NET
        NET tmpNet;
        unsigned tmpNetId = _sizer->_ckt->g_nets[0].size();
        tmpNet.name = netName;
        _sizer->_ckt->net2id.insert(
            pair< string, unsigned >(netName, tmpNetId));

        // NET -- PINS
        // here are the gates connected to the net
        NetPinIterator* instTerms_iter = network->pinIterator(net);
        int net_pin_num = 0;
        while(instTerms_iter->hasNext()) {
            net_pin_num++;
            auto instGatePin = instTerms_iter->next();
            Instance* instGate = network->instance(instGatePin);

            string instGateName = network->name(instGate);

            char tmpName[2000];
            strcpy(tmpName, instGateName.c_str());

            int gateId = -1;

            if(_sizer->_ckt->cell2id.find(tmpName) !=
               _sizer->_ckt->cell2id.end())
                gateId = _sizer->_ckt->cell2id[tmpName];

            if(gateId >= _sizer->_ckt->g_cells.size() || gateId < 0)
                printf("error gate id %d\n", gateId);

            CELL& cell = _sizer->_ckt->g_cells[gateId];

            string pin_name = network->name(instGatePin);

            size_t found = pin_name.find_last_of("/");
            string termName = pin_name.substr(found + 1);

            // NEW PIN
            PIN tmpPin;
            tmpPin.id = _sizer->_ckt->g_pins.size();
            tmpPin.name = termName.c_str();
            tmpPin.net = _sizer->_ckt->g_nets[0].size();
            tmpPin.owner = gateId;

            _sizer->_ckt->pin2id.insert(
                pair< string, unsigned >(pin_name, tmpPin.id));
            _sizer->_ckt->g_pins.push_back(tmpPin);

            PortDirection* dir = network->direction(instGatePin);

            if(dir->isAnyOutput()) {
                cell.outpins.push_back(tmpPin.id);
                tmpNet.inpin = tmpPin.id;
                cell.pinchar.insert(
                    pair< string, unsigned >(tmpPin.name, tmpPin.id));
            }
            else {
                cell.inpins.push_back(tmpPin.id);
                cell.pinchar.insert(
                    pair< string, unsigned >(tmpPin.name, tmpPin.id));
                tmpNet.outpins.push_back(tmpPin.id);
            }

        }  // NET -- INST PINS

        // printf("Net %s has %d pins\n", netName.c_str(), net_pin_num);
        // PI/PO
        // InstancePinIterator
        // *top_level_pins_iter=network->pinIterator(network->topInstance());
        NetConnectedPinIterator* top_level_pins_iter =
            network->connectedPinIterator(net);

        while(top_level_pins_iter->hasNext()) {
            auto term = top_level_pins_iter->next();

            if(network->isTopLevelPort(term)) {
                string termName = network->name(term);
                char tmpName[120];
                strcpy(tmpName, termName.c_str());

                // NEW PIN
                PIN tmpPin2;
                tmpPin2.id = _sizer->_ckt->g_pins.size();
                tmpPin2.name = tmpName;
                tmpPin2.net = tmpNetId;
                tmpPin2.owner = UINT_MAX;

                PortDirection* dir_term = network->direction(term);

                if(dir_term->isAnyOutput()) {
                    _sizer->_ckt->outdelays.insert(
                        pair< string, double >(tmpName, 0.0));
                    _sizer->_ckt->POs.push_back(tmpPin2.id);
                    tmpNet.outpins.push_back(tmpPin2.id);
                    tmpPin2.isPO = true;
                }
                else {
                    _sizer->_ckt->indelays.insert(
                        pair< string, double >(tmpName, 0.0));
                    _sizer->_ckt->PIs.push_back(tmpPin2.id);
                    tmpNet.inpin = tmpPin2.id;
                    tmpPin2.isPI = true;
                }
                _sizer->_ckt->pin2id.insert(
                    pair< string, unsigned >(tmpName, tmpPin2.id));
                _sizer->_ckt->g_pins.push_back(tmpPin2);
            }
        }  // PI/PO END

        for(unsigned corner = 0; corner < _sizer->numCorners; ++corner) {
            _sizer->_ckt->g_nets[corner].push_back(tmpNet);
        }

    }  // NET ITERATION END

    // SDC Copy runing
    // TODO:
    int mode = 0;
    auto* sdc = _sta->sdc();
    auto* clock = sdc->clocks()->at(mode);
    _sizer->clk_name[mode] = clock->name();
    _sizer->clk_period[mode] = clock->period() / _sizer->time_unit;

    auto* clk_pin = *(clock->pins().begin());
    auto* concrete_pin = reinterpret_cast< const sta::ConcretePin* >(clk_pin);
    std::string clock_port_name = concrete_pin->name();
    _sizer->clk_port[mode] = clock_port_name;

    float slew = 0.0;
    bool is_exist = false;
    clock->slew(sta::RiseFall::rise(), MinMax::max(), slew, is_exist);
    slew = slew / _sizer->time_unit;
    _sizer->inrtran[mode].insert(
        std::pair< unsigned, double >(pin2id[clock_port_name], slew));

    clock->slew(sta::RiseFall::fall(), MinMax::max(), slew, is_exist);
    _sizer->inftran[mode].insert(
        std::pair< unsigned, double >(pin2id[clock_port_name], slew));
    auto input_delay_iterator = sdc->inputDelays().begin();
    // auto* input_delay_iterator = ;
    while(input_delay_iterator != sdc->inputDelays().end()) {
        auto* input_delay = *input_delay_iterator;
        string port_name =
            reinterpret_cast< const sta::ConcretePin* >(input_delay->pin())
                ->name();
        auto delays = input_delay->delays();
        // do not care min delay
        auto max_rise_delay =
            delays->value(sta::RiseFall::rise(), MinMax::max());
        auto max_fall_delay =
            delays->value(sta::RiseFall::fall(), MinMax::max());

        _sizer->indelays[mode].insert(
            pair< string, double >(port_name, max_rise_delay));
        _sizer->indelays[mode].insert(
            pair< string, double >(port_name, max_fall_delay));
        input_delay_iterator++;
    }

    auto out_delay_iterator = sdc->outputDelays().begin();
    while(out_delay_iterator != sdc->outputDelays().end()) {
        auto* output_delay = *out_delay_iterator;
        string port_name =
            reinterpret_cast< const sta::ConcretePin* >(output_delay->pin())
                ->name();
        auto delays = output_delay->delays();

        // do not care min delay
        auto max_rise_delay =
            delays->value(sta::RiseFall::rise(), MinMax::max());
        auto max_fall_delay =
            delays->value(sta::RiseFall::fall(), MinMax::max());

        _sizer->outdelays[mode].insert(
            pair< string, double >(port_name, max_rise_delay));
        _sizer->outdelays[mode].insert(
            pair< string, double >(port_name, max_fall_delay));
        out_delay_iterator++;
    }

    const PortExtCapMap& port_cap_map = sdc->port_ext_cap_maps_.at(0);
    for(auto [port, cap] : port_cap_map) {
        std::string port_name =
            reinterpret_cast< const sta::ConcretePort* >(port)->name();
        double load =
            cap->pinCap()->value(sta::RiseFall::rise(), MinMax::max());
        assert(load < 1e31);
        g_pins[pin2id[port_name]].cap = load;
    }

    auto& input_slew_map = sdc->input_drive_map_;

    for(auto [port, drive] : input_slew_map) {
        std::string port_name =
            reinterpret_cast< const sta::ConcretePort* >(port)->name();
#ifdef DRIVER_CELL
        std::string driverSize;
        if(!drive->hasDriveCell(sta::RiseFall::rise(), MinMax::max())) {
            continue;
        }
        else {
            std::string driverSize =
                drive->driveCell(sta::RiseFall::rise(), MinMax::max())
                    ->cell()
                    ->name();
        }
        unsigned driverInPin = g_pins[pin2id[port_name]].lib_pin;
        unsigned driverOutPin = g_pins[pin2id[port_name]].lib_pin;
        LibCellInfo& lib_cell = _sizer->libs[mode].find(driverSize)->second;

        // JLTimingArc: add driverOutPin info
        std::map< unsigned, LibPinInfo >::iterator it;
        for(it = lib_cell.pins.begin(); it != lib_cell.pins.end(); ++it) {
            if((it->second).isInput == true) {
                driverInPin = lib_cell.lib_pin2id_map[(it->second).name];
            }
            if((it->second).isOutput == true) {
                driverOutPin = lib_cell.lib_pin2id_map[(it->second).name];
            }
        }
        _sizer->drivers[mode].insert(
            std::pair< unsigned, string >(pin2id[port_name], driverSize));
        _sizer->driverInPins[mode].insert(
            std::pair< unsigned, unsigned >(pin2id[port_name], driverInPin));
        // JLTimingArc: add driverOutPin info
        _sizer->driverOutPins[mode].insert(
            std::pair< unsigned, unsigned >(pin2id[port_name], driverOutPin));
#endif
        float slew = 0.0;
        bool is_exist = false;
        drive->slew(sta::RiseFall::rise(), sta::MinMax::max(), slew, is_exist);
        slew /= _sizer->time_unit;
        _sizer->inrtran[mode].insert(
            std::pair< unsigned, double >(pin2id[port_name], slew));

        drive->slew(sta::RiseFall::fall(), sta::MinMax::max(), slew, is_exist);
        slew /= _sizer->time_unit;
        _sizer->inftran[mode].insert(
            std::pair< unsigned, double >(pin2id[port_name], slew));
    }
}

void Circuit::readSpef_opensta(sta::Sta* _sta) {
    cout << "[opensta] reading spef..." << endl;
    Parasitics* parasitics = _sta->parasitics();

    Corner* _corner = _sta->corners()->corners()[0];
    const MinMax* cnst_min_max;
    ParasiticAnalysisPt* ap;
    // _sta->corners()->makeParasiticAnalysisPtsMinMax();
    cnst_min_max = MinMaxAll::max()->asMinMax();
    ap = _corner->findParasiticAnalysisPt(cnst_min_max);

    double totcap = 0.0;
    int corner = 0;

    for(unsigned i = 0; i < g_nets[corner].size(); ++i) {
        string netNameStr = g_nets[corner][i].name;

        g_nets[corner][net2id[netNameStr]].cap = totcap;
        vector< SUB_NODE >* subNodeVecPtr =
            &g_nets[corner][net2id[netNameStr]].subNodeVec;
        string name;
        string attr;

        int node_index = 1;
        std::vector< SUB_NODE >::iterator subNodeIter;

        Net* net = _sta->network()->findNet(netNameStr.c_str());
        NetConnectedPinIterator* connPinIter =
            _sta->network()->connectedPinIterator(net);

        SUB_NODE sn;
        sn.isSink = false;
        sn.id = 0;
        sn.pinId = -1;
        subNodeVecPtr->push_back(sn);

        while(connPinIter->hasNext()) {
            auto connPin = connPinIter->next();

            string pin_name = _sta->network()->name(connPin);
            PortDirection* dir = _sta->network()->direction(connPin);
            // if(pin_name ==
            //    "u_NV_NVDLA_cmac_u_core_u_mac_0_mul_128_55_g7067/CON") {
            //     puts("debug debug");
            // }
            if(strcmp(pin_name.c_str(), string("SE").c_str()) == 0 ||
               strcmp(pin_name.c_str(), string("SI").c_str()) == 0 ||
               strcmp(pin_name.c_str(), string("SO").c_str()) == 0 ||
               strcmp(pin_name.c_str(), string("HRESETn").c_str()) == 0)
                continue;

            SUB_NODE sn;

            // Input
            if(dir->isInput()) {
                readSpefChangePinName(pin_name);
                subNodeVecPtr->at(0).pinId = pin2id[pin_name];
                g_pins[pin2id[pin_name]].spef_pin = 0;
                node2id[pin_name] = 0;
            }
            else {
                // Output
                sn.isSink = true;
                sn.id = node_index++;
                readSpefChangePinName(pin_name);

                node2id[pin_name] = sn.id;

                sn.pinId = pin2id[pin_name];
                g_pins[sn.pinId].spef_pin = sn.id;

                subNodeVecPtr->push_back(sn);
            }
            Parasitic* para = parasitics->findParasiticNetwork(
                connPin, ap);  // 对应net上的Parasitic
            if(para == nullptr) {
                printf(
                    "Error, Net don't have Parasitic. Net name :%s, pin name: "
                    "%s, \n",
                    netNameStr.c_str(), pin_name.c_str());
                continue;
            }
            ConcreteParasitic* conc_para =
                static_cast< ConcreteParasitic* >(para);
            sta::ConcreteParasiticNetwork* conc_net_para =
                static_cast< sta::ConcreteParasiticNetwork* >(conc_para);

            ParasiticNode* para_node =
                conc_net_para->findParasiticNode(connPin);
            auto paraDevIter = conc_net_para->resistors().begin();
            while(paraDevIter != conc_net_para->resistors().end()) {
                ParasiticResistor* paraDev = *paraDevIter;

                ParasiticNode* other_node =
                    parasitics->otherNode(paraDev, para_node);

                string fromNodeNameStr = parasitics->name(para_node);
                string toNodeNameStr = parasitics->name(other_node);
                readSpefChangePinName(fromNodeNameStr);
                readSpefChangePinName(toNodeNameStr);
                std::map< string, int >::const_iterator node2IdIter1 =
                    node2id.find(fromNodeNameStr);
                std::map< string, int >::const_iterator node2IdIter2 =
                    node2id.find(toNodeNameStr);
                unsigned index1, index2;

                float value = parasitics->value(paraDev);

                // RES
                // if (parasitics->isResistor(paraDev)) {
                SUB_NODE sn;
                sn.cap = 0.0;
                sn.id = node_index++;

                if(node2id.find(fromNodeNameStr) == node2id.end()) {
                    node2id[fromNodeNameStr] = sn.id;
                    subNodeVecPtr->push_back(sn);
                    index1 = sn.id;
                }
                else {
                    index1 = node2IdIter1->second;
                }
                if(node2id.find(toNodeNameStr) == node2id.end()) {
                    node2id[toNodeNameStr] = sn.id;
                    subNodeVecPtr->push_back(sn);
                    index2 = sn.id;
                }
                else {
                    index2 = node2IdIter2->second;
                }

                if(index1 < subNodeVecPtr->size() &&
                   index2 < subNodeVecPtr->size()) {
                    subNodeVecPtr->at(index1).adj.push_back(index2);
                    subNodeVecPtr->at(index2).adj.push_back(index1);
                    subNodeVecPtr->at(index1).res.push_back(value);
                    subNodeVecPtr->at(index2).res.push_back(value);
                }
                paraDevIter++;
            }

            sta::ConcreteParasiticSubNodeMap* sub_nodes =
                conc_net_para->subNodes();
            auto sub_nodes_iter = sub_nodes->begin();

            // CAP
            while(sub_nodes_iter != sub_nodes->end()) {
                ConcreteParasiticNode* node = sub_nodes_iter->second;

                string fromNodeNameStr = node->name(_sta->network());
                std::map< string, int >::const_iterator node2IdIter1 =
                    node2id.find(fromNodeNameStr);
                float value_cap = node->capacitance();

                if(node2IdIter1 != node2id.end()) {
                    subNodeVecPtr->at(node2IdIter1->second).cap = value_cap;
                }
                else {
                    SUB_NODE sn;
                    sn.cap = value_cap;
                    sn.id = node_index++;
                    node2id[fromNodeNameStr] = sn.id;
                    subNodeVecPtr->push_back(sn);
                }
                sub_nodes_iter++;
            }
        }

        g_nets[corner][net2id[netNameStr]].cap = totcap;
    }
}

void Circuit::readSpefChangePinName(string& pin_name) {
    size_t index = pin_name.find(":");
    if(index != string::npos) {
        pin_name.replace(index, 1, "/");
    }
}

void evalTclInitForLibrary(Tcl_Interp* interp, const char* inits[]) {
    size_t length = 0;
    for(const char** e = inits; *e; e++) {
        const char* init = *e;
        length += strlen(init);
    }
    char* unencoded = new char[length / 3 + 1];
    int line = 0;
    char* u = unencoded;
    for(const char** e = inits; *e; e++) {
        // printf("%d\n", line);
        const char* init = *e;
        size_t init_length = strlen(init);
        for(const char* s = init; s < &init[init_length]; s += 3) {
            char code[4] = {s[0], s[1], s[2], '\0'};
            char ch = atoi(code);
            *u++ = ch;
        }
        line++;
    }
    *u = '\0';
    if(Tcl_Eval(interp, unencoded) != TCL_OK) {
        printf("%s\n", unencoded);
        printf("TCL init script: %s.\n", Tcl_GetStringResult(interp));
    };
    delete[] unencoded;
}
