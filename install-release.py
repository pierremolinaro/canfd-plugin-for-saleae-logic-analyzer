#! /usr/bin/env python
# -*- coding: UTF-8 -*-

#------------------------------------------------------------------------------

import sys, os, subprocess

#------------------------------------------------------------------------------

#--- Get script absolute path
scriptDir = os.path.dirname (os.path.abspath (sys.argv [0]))
#--- Build Analyzer
childProcess = subprocess.Popen (["python", "build_analyzer.py"], cwd=scriptDir)
if childProcess.poll () == None :
  childProcess.wait ()
if childProcess.returncode != 0 :
  sys.exit (childProcess.returncode)
#--- Build Analyzer
childProcess = subprocess.Popen ([
  "install_name_tool",
  "-change",
  "@executable_path/libAnalyzer.dylib",
  "@rpath/libAnalyzer.dylib",
  "release/libCANFDMolinaroAnalyzer.dylib"
], cwd=scriptDir)
if childProcess.poll () == None :
  childProcess.wait ()
if childProcess.returncode != 0 :
  sys.exit (childProcess.returncode)
#--- Copy analyzer
childProcess = subprocess.Popen ([
  "cp",
  "release/libCANFDMolinaroAnalyzer.dylib",
  os.path.expanduser ("~/Documents/customSaleaeLogicAnalyzers")
], cwd=scriptDir)
if childProcess.poll () == None :
  childProcess.wait ()
if childProcess.returncode != 0 :
  sys.exit (childProcess.returncode)

#------------------------------------------------------------------------------
