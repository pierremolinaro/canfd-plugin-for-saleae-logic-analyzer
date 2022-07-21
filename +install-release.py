#! /usr/bin/env python
# -*- coding: UTF-8 -*-

#------------------------------------------------------------------------------

import sys, os, subprocess

#------------------------------------------------------------------------------

#--- Get script absolute path
scriptDir = os.path.dirname (os.path.abspath (sys.argv [0]))
os.chdir (scriptDir)
#---
runInit = False
if not os.path.exists ("build") :
  runInit = True
  returncode = subprocess.call (["mkdir", "build"])
  if returncode != 0 :
    sys.exit (returncode)
#--- Change directory
os.chdir ("build")
#--- Run init
if runInit :
  result = subprocess.call (["cmake", ".."])
  if returncode != 0 :
    sys.exit (returncode)
#--- Build Analyzer
returncode = subprocess.call (["cmake", "--build", "."])
if returncode != 0 :
  sys.exit (returncode)
#--- Copy analyzer
returnCode = subprocess.call ([
  "cp",
  "Analyzers/libCANFDMolinaroAnalyzer.so",
  os.path.expanduser ("~/Documents/customSaleaeLogicAnalyzers")
])
if returncode != 0 :
  sys.exit (returncode)
#--- Remove attributes
returnCode = subprocess.call ([
  "xattr", "-r", "-d", "com.apple.quarantine",
  os.path.expanduser ("~/Documents/customSaleaeLogicAnalyzers/libCANFDMolinaroAnalyzer.so")
])
if returncode != 0 :
  sys.exit (returncode)

#------------------------------------------------------------------------------
