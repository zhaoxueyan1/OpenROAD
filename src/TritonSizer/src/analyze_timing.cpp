///////////////////////////////////////////////////////////////////////////////
// Authors: Hyein Lee and Jiajia Li
//          (Ph.D. advisor: Andrew B. Kahng)
//
//          Many subsequent improvements were made by Minsoo Kim
//          leading up to the initial release.
//
// BSD 3-Clause License
//
// Copyright (c) 2018, The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

// ****************************************************************************
// analyzeTiming.cpp
//
// Make contact with PT/ETS/SOCE Tcl server, pass commands,
// get results and report
// ****************************************************************************

#include "analyze_timing.h"
#include <stdlib.h>
#include <sys/time.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include "sta/Sta.hh"
#include "utils.h"
#include "sizer.h"
#include <tcl8.6/tcl.h>
#include <tcl8.6/tclDecls.h>

designTiming::designTiming() {
    program = PT;
    pt_time = 0.0;
}
designTiming::designTiming(ServerProg _program) {
    program = _program;
    pt_time = 0.0;
}
designTiming::~designTiming() {
}

void designTiming::testTCL() {
    _interpreter = Tcl_CreateInterp();
    _tclInputString = "puts \"TCL TEST";
    //_tclExpression = (char *)_tclInputString.c_str();
    Tcl_Eval(_interpreter, _tclExpression);
}

void designTiming::initializeServerContact(string clientName) {
    _interpreter = Tcl_CreateInterp();
    ifstream _clientTcl(clientName.c_str());
    if(!_clientTcl) {
        cerr << "Fatal error: cannot proceed without *client.tcl file" << endl;
        exit(1);
    }
    _tclInputString = "source " + clientName;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    Tcl_Eval(_interpreter, _tclExpression);

    _tclInputString = "InitClient";
    //_tclExpression = (char *)_tclInputString.c_str();
    cout << "InitClient" << endl;
    Tcl_Eval(_interpreter, _tclExpression);
    pt_time += cpuTime() - begin;
}

void designTiming::closeServerContact() {
    _tclInputString = "CloseClient";
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    Tcl_Eval(_interpreter, _tclExpression);
    Tcl_DeleteInterp(_interpreter);
    pt_time += cpuTime() - begin;
}

// void designTiming::getCellDelay(double &delay, string &riseFall,
//                                 string cellInPin, string cellOutPin) {
//     _tclInputString =
//         "gate_delay " + cellInPin + " " + cellOutPin ;
//     // cout << _tclInputString << endl;
//     //_tclExpression = (char *)_tclInputString.c_str();
//     double begin = cpuTime();
//     sta::evalTclString( _tclExpression);

//     pt_time += cpuTime() - begin;
//     string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
//     float temp1;
//     char temp2[128];
//     sscanf(_tclAnswer.c_str(), "%f%s", &temp1, &temp2);
//     delay = temp1;
//     riseFall = temp2;

//     // cout << delay << " " << riseFall << " " << endl;
// }

void designTiming::getCellDelay(double &rise_delay, double &fall_delay,
                                string cellInPin, string cellOutPin) {
    _tclInputString = "gate_delay " + cellInPin + " " + cellOutPin;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);

    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp1;
    float temp2;
    sscanf(_tclAnswer.c_str(), "%f%f", &temp1, &temp2);
    rise_delay = temp1;
    fall_delay = temp2;

    // cout << delay << " " << riseFall << " " << endl;
}

void designTiming::getFFDelay(double &rdelay, double &fdelay,
                              string cellOutPin) {
    _tclInputString = "PtGetFFDelay " + cellOutPin;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    pt_time += cpuTime() - begin;
    float temp1;
    float temp2;
    sscanf(_tclAnswer.c_str(), "%f%f", &temp1, &temp2);
    rdelay = temp1;
    fdelay = temp2;
}

void designTiming::getNetDelay(double &delay, string sourcePinName,
                               string sinkPinName) {
    _tclInputString = "report_wire_delay " + sourcePinName + " " + sinkPinName;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    pt_time += cpuTime() - begin;
    float temp;
    sscanf(_tclAnswer.c_str(), "%f", &temp);
    delay = temp;
}

void designTiming::getInputSlew(double &riseSlew, double &fallSlew,
                                string pinName) {
    _tclInputString = "PtGetCellInputSlew " + pinName;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp1;
    float temp2;
    sscanf(_tclAnswer.c_str(), "%f%f", &temp1, &temp2);
    riseSlew = temp1;
    fallSlew = temp2;
}

string designTiming::_convertToString(double x) {
    std::ostringstream o;
    o << x;
    return o.str();
}

double designTiming::_convertToDouble(const string &s) {
    std::istringstream i(s);
    double x;
    i >> x;
    if(s == "INFINITY") {
        x = std::numeric_limits< double >::infinity();
    }
    return x;
}

int designTiming::_convertToInt(const string &s) {
    std::istringstream i(s);
    int x;
    i >> x;
    return x;
}

double designTiming::getWorstSlackHold(string _clkName) {
    if(program == PT) {
        _tclInputString = "PtWorstHoldSlack " + _clkName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWorstHoldSlack " + _clkName;
    }
    else if(program == OS) {
        _tclInputString = "OSWorstHoldSlack " + _clkName;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double _pathSlack = _convertToDouble(_answerStr);
    return (_pathSlack);
}

double designTiming::getWorstSlack(string _clkName) {
    if(program == PT) {
        _tclInputString = "PtWorstSlack " + _clkName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWorstSlack " + _clkName;
    }
    else if(program == OS) {
        _tclInputString = "OSWorstSlack " + _clkName;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double _pathSlack = _convertToDouble(_answerStr);
    return (_pathSlack);
}

double designTiming::getTotPower() {
    if(program == PT) {
        _tclInputString = "PtTotalPower";
    }
    else if(program == ETS) {
        _tclInputString = "EtsTotalPower";
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    // cout << _tclInputString << endl;
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    // cout << _tclAnswer << endl;
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double _totPower = _convertToDouble(_answerStr);
    return (_totPower);
}

double designTiming::getLeakPower() {
    if(program == PT) {
        _tclInputString = "PtLeakPower";
    }
    else if(program == ETS) {
        _tclInputString = "EtsLeakPower";
    }
    else {
        return 0;
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    // cout << _tclInputString << endl;
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double _leakPower = _convertToDouble(_answerStr);
    return (_leakPower);
}

double designTiming::getTNS(string _clkName) {
    if(program == PT) {
        _tclInputString = "PtGetTNS " + _clkName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetTNS " + _clkName;
    }
    else if(program == OS) {
        _tclInputString = "OSGetTNS";
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double _pathSlack = _convertToDouble(_answerStr);
    return (_pathSlack);
}

void designTiming::getTranVio(double &tot, double &max, int &num) {
    if(program == PT) {
        _tclInputString = "PtGetTranVio ";
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetTranVio ";
    }
    else if(program == OS) {
        _tclInputString = "OSGetTranVio ";
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);

    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp1;
    float temp2;
    int temp3;
    sscanf(_tclAnswer.c_str(), "%f%f%d", &temp1, &temp2, &temp3);
    tot = temp1;
    max = temp2;
    num = temp3;
}

double designTiming::getTNSHold(string _clkName) {
    if(program == PT) {
        _tclInputString = "PtGetTNSHold " + _clkName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetTNSHold " + _clkName;
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double _pathSlack = _convertToDouble(_answerStr);
    return (_pathSlack);
}

bool designTiming::sizeCell(string cellInstance, string cellMaster) {
    if(program == PT) {
        _tclInputString = "PtSizeCell " + cellInstance + " " + cellMaster;
    }
    else if(program == ETS) {
        _tclInputString = "EtsSizeCell " + cellInstance + " " + cellMaster;
    }
    else if(program == OS) {
        _tclInputString = "OSSizeCell " + cellInstance + " " + cellMaster;
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    int _returnStatus = _convertToInt(_answerStr);
    if(_returnStatus == 0) {
        cerr << "Fatal error: size_cell failed; check the status on ptserver"
             << endl;
        return false;
    }
    return true;
}

bool designTiming::writeECOChange(string filename) {
    if(program == PT) {
        _tclInputString = "write_change -format ptsh -output  " + filename;
    }
    else if(program == ETS) {
        _tclInputString = "write_eco -format ets -output " + filename;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

double designTiming::getCellSlack(string CellName) {
    if(CellName == "")
        return ((double)0);
    else {
        if(program == PT || program == ETS) {
            _tclInputString = "PtCellSlack " + CellName;
        }
        //_tclExpression = (char *)_tclInputString.c_str();
        double begin = cpuTime();
        sta::evalTclString(_tclInputString);
        pt_time += cpuTime() - begin;

        string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
        string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
        double _setupSlack = _convertToDouble(_answerStr);
        return (_setupSlack);
    }
}

bool designTiming::loadDesign(string benchname) {
    _tclInputString =
        "redirect pt.loadDesign.log {source pt." + benchname + ".tcl}";
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::updateSize(string filename) {
    //_tclInputString = "redirect pt.updateSize.log {source " +
    // filename+"}";
    _tclInputString = "source " + filename;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::checkSize(string filename) {
    if(program == PT) {
        _tclInputString = "PtGetCurSize " + filename;
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetCurSize " + filename;
    }
    else if(program == OS) {
        _tclInputString = "OSGetCurSize " + filename;
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::checkServer() {
    _tclInputString = "checkServer";
    // cout << "checkServer\" " << this << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

void designTiming::getPinSlack(double &riseSlack, double &fallSlack,
                               string pinName) {
    if(program == PT) {
        _tclInputString = "PtGetPinSlack " + pinName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetPinSlack " + pinName;
    }
    else if(program == OS) {
        _tclInputString = "OSGetPinSlack " + pinName;
    }
    //_tclExpression = (char *)_tclInputString.c_str();

    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp1;
    float temp2;
    sscanf(_tclAnswer.c_str(), "%f%f", &temp1, &temp2);
    riseSlack = temp1;
    fallSlack = temp2;
}

void designTiming::getPinMinSlack(double &riseSlack, double &fallSlack,
                                  string pinName) {
    if(program == PT) {
        _tclInputString = "PtGetPinMinSlack " + pinName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetPinMinSlack " + pinName;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();

    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp1;
    float temp2;
    sscanf(_tclAnswer.c_str(), "%f%f", &temp1, &temp2);
    riseSlack = temp1;
    fallSlack = temp2;
    // cout << pinName << " " << riseSlack << " " << fallSlack << endl;
}

void designTiming::getPinTran(double &riseTran, double &fallTran,
                              string pinName) {
    _tclInputString = "OSGetPinTran " + pinName;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp1;
    float temp2;
    sscanf(_tclAnswer.c_str(), "%f%f", &temp1, &temp2);
    riseTran = temp1;
    fallTran = temp2;
    // cout << pinName << " " << riseSlack << " " << fallSlack << endl;
}

bool designTiming::writePinSlack(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinSlack " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinSlack " + infile + " " + outfile;
    }
    else if(program == OS) {
        _tclInputString = "OSWritePinSlack " + infile + " " + outfile;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::writePinMinSlack(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinMinSlack " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinMinSlack " + infile + " " + outfile;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::writeMaxTranConst(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinMaxTranConst " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinMaxTranConst " + infile + " " + outfile;
    }
    else {
        _tclInputString = "OSWritePinMaxTranConst " + infile + " " + outfile;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

// JLPWR
bool designTiming::writePinToggleRate(string infile, string outfile,
                                      string clock_period) {
    if(program == PT) {
        _tclInputString = "PtWritePinToggleRate " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinToggleRateWithClock " + infile + " " +
                          outfile + " " + clock_period;
    }
    else if(program == OS) {
        _tclInputString = "OSWritePinToggleRate " + infile + " " + outfile;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::writePinToggleRate(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinToggleRate " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinToggleRate " + infile + " " + outfile;
    }
    else if(program == OS) {
        _tclInputString = "OSWritePinToggleRate " + infile + " " + outfile;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

// JLPWR
void designTiming::getPinToggleRate(double &toggleRate, string pinName) {
    if(program == PT || program == ETS) {
        _tclInputString = "PtGetPinToggleRate " + pinName;
    }
    else {
        _tclInputString = "OSGetPinToggleRate " + pinName;
        toggleRate = 0;
        return;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();

    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp;
    sscanf(_tclAnswer.c_str(), "%f", &temp);
    toggleRate = temp;
}

bool designTiming::writePinTran(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinTran " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "PtWritePinTran " + infile + " " + outfile;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::writePinAll(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinAll " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinAll " + infile + " " + outfile;
    }
    else if(program == OS) {
        _tclInputString = "OSWritePinAll " + infile + " " + outfile;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

void designTiming::getPinArrival(double &riseArrival, double &fallArrival,
                                 string pinName) {
    _tclInputString = "PtGetPinArrival " + pinName;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp1;
    float temp2;
    sscanf(_tclAnswer.c_str(), "%f%f", &temp1, &temp2);
    riseArrival = temp1;
    fallArrival = temp2;
    // cout << pinName << " " << riseSlack << " " << fallSlack << endl;
}

double designTiming::getRiseSlack(string PinName) {
    if(program == PT || program == ETS) {
        _tclInputString = "PtGetRiseSlack " + PinName;
    }
    else {
        _tclInputString = "OSGetRiseSlack " + PinName;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double slack = _convertToDouble(_answerStr);
    return slack;
}

double designTiming::getFallSlack(string PinName) {
    if(program == PT || program == ETS) {
        _tclInputString = "PtGetFallSlack " + PinName;
    }
    else {
        _tclInputString = "OSGetFallSlack " + PinName;
    }

    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double slack = _convertToDouble(_answerStr);
    return slack;
}

double designTiming::getRiseTran(string PinName) {
    if(program == PT) {
        _tclInputString = "PtGetRiseTran " + PinName;
    }
    else {
        _tclInputString = "OSGetRiseTran " + PinName;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double tran = _convertToDouble(_answerStr);
    return tran;
}

double designTiming::getFallTran(string PinName) {
    if(program == PT) {
        _tclInputString = "PtGetFallTran " + PinName;
    }
    else {
        _tclInputString = "OSGetFallTran " + PinName;
    }

    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double tran = _convertToDouble(_answerStr);
    return tran;
}

double designTiming::getRiseArrival(string PinName) {
    _tclInputString = "PtGetRiseArrival " + PinName;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double val = _convertToDouble(_answerStr);
    return val;
}

double designTiming::getFallArrival(string PinName) {
    _tclInputString = "PtGetFallArrival " + PinName;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double val = _convertToDouble(_answerStr);
    return val;
}

double designTiming::getCeff(string PinName) {
    if(program == PT) {
        _tclInputString = "PtGetCeff " + PinName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetCeff " + PinName;
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double val = _convertToDouble(_answerStr);
    return val;
}

bool designTiming::writePinCeff(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinCeff " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinCeff " + infile + " " + outfile;
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::runECO(unsigned mode) {
    if(program != PT)
        return false;

    string cmd = "";
    if(mode == 0) {
        cmd = "fix_eco_timing -type setup -methods size_cell";
    }
    else if(mode == 1) {
        cmd = "fix_eco_drc -type max_transition -methods size_cell";
    }
    else if(mode == 2) {
        cmd = "fix_eco_drc -type max_capacitance -methods size_cell";
    }

    _tclInputString = "" + cmd;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    if(_answerStr == "1")
        return true;
    else
        return false;
}

string designTiming::getLibCell(string CellName) {
    _tclInputString = "PtGetLibCell " + CellName;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    return (_answerStr);
}

void designTiming::Exit() {
    _tclInputString = "exitServer";
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
}

string designTiming::doOneCmd(string command) {
    _tclInputString = "" + command;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    sta::evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    return _answerStr;
}
