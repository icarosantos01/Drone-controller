//
// File: _coder_main_codegen_info.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

// Include Files
#include "_coder_main_codegen_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

// Function Declarations
static const mxArray *emlrtMexFcnResolvedFunctionsInfo();

// Function Definitions
//
// Arguments    : void
// Return Type  : const mxArray *
//
static const mxArray *emlrtMexFcnResolvedFunctionsInfo()
{
  const mxArray *nameCaptureInfo;
  const char_T *data[9]{
      "789ced9a4d73d24018c78353b533be7172f44b386d055af5e0d040696d819497da179c26"
      "848506922c4d1604c743bf81574f4e0f1e9ce98cf6e807f03378f2ea"
      "454fbd79b401b690ccec04489a960d7be876e7df677f4f9f947f9e4dca04d692018661ee"
      "33ddf1ef6577bed75b077bf30dc63cac7aa037dfb5acf1b8c9cc98e2",
      "b0fea1378b5045a085ba0b5550c04564092a922aa828d7ae0346033a949ba0d451ca920c"
      "729202b2838b94b1525606a48b852119dfb30740ac651b0aa31de8fd"
      "0ce5c1c5453d3e127edf9921eb1121d42368d1f7e26fd8e785bc0e34bd2095f5c8622119"
      "cd6d44970b9c06ab40447a211a0f85e70a8a20a9fb222c810a509f28",
      "fd3c8f1ce6f9d8264face734c148076a6d4e1654156838999e8ef3e1c7ccc73a48f9e081"
      "795fc6e4e1fd13363cacefb1c35d27592a1688b53abf7076757a3064"
      "ded6b9fff3b39df9f3e9af8ee415affdfbec8f973c3cae8ad722ec37ecdfdd43022f68d1"
      "5bbbe5583aa945b9741a44cba126bbbb5d0b27fa7970361cbb3c18c2",
      "daabfd27f5f35b7798b7f57e68cd1beb7528b7c3fb3aa8284035ee924e7d16f36e11f95d"
      "a5041b4519f479c70e792f883cb33eecfdd0b84ea6da183745c63b3f"
      "d0bf7aebaf7f7fbce7bde4e141bbbfe6c1667473bbbc34170bcdad2cd5decd5736338d18"
      "3dfe3a297dac577e1ad3a00a067a57affc54475a434457e0a723dcf7",
      "4cb5a1dc4f7f364e142f7978d0eea7eb89eaebad6224b1d8ccbf5a85db873be9524860e9"
      "f1539e107f59e75ba7be487a2e13b4e8158032a00c34a08a60907fe4"
      "90eff6f385ef0ef3c9dbe483f5bdb43b078680a5b07ef11bda799ef5a7bba9ad48221bab"
      "a6560ff3d54a9b139e722245e77f9e103fe97eca415d42125459a4c9",
      "6ef29df6b9a70ef96b367cac8fe49f96c633602d1ee31f5fa19d37f54d77f63f26c44fda"
      "f971d87c672ceb7ebe5d8595a5ba9bbc519f8f3a7dceb240e499f551"
      "9e8f1a35e95d066acff1df3e054de795cbe6e141bb7f86b8e51d54d362ac22b3cfaa5bd2"
      "e25b9812e3f4f8274f889ff4be338a90841a2530ed3bc7e83badc563",
      "fce32bb4f3a67da73bfbf384f8ebea9b776c78584f4204b5a4d492d48aab7ca7bee9f4ff"
      "23566df85877e49b03c5f38b9fd0ce9bfaa53bfbf384f849f7cb7cbd"
      "24209045e75f5ce5fbc22f078ae7173fa19d37f54b77f6e709f197e597e3f2f0feb789bc"
      "ae126f0a723c9da4c61f63367cac3bf2475c34c63ffe413b6fea8fee",
      "ec7f4c88a7f5bd8f9dbf66b89d850c4413fbde274ce499f551defbf46ad2b912b4bef779"
      "747652f4928707edfe594eea70bdba312f46e3a2ca2e271bd554314f"
      "917ff284f8ebda5fce5ad67d5e574900643a474e7a7f19b7e163dd517f898bc6f8c73f68"
      "e74dfb4b67fbff07e0abf873",
      ""};
  nameCaptureInfo = nullptr;
  emlrtNameCaptureMxArrayR2016a(&data[0], 16400U, &nameCaptureInfo);
  return nameCaptureInfo;
}

//
// Arguments    : void
// Return Type  : mxArray *
//
mxArray *emlrtMexFcnProperties()
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9]{"Version",
                                 "ResolvedFunctions",
                                 "Checksum",
                                 "EntryPoints",
                                 "CoverageInfo",
                                 "IsPolymorphic",
                                 "PropertyList",
                                 "UUID",
                                 "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8]{
      "Name",     "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "FullPath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 0);
  emlrtSetField(xEntryPoints, 0, "Name", emlrtMxCreateString("main_codegen"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(0.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString(
          "C:\\Users\\ifs67\\MATLAB\\Projects\\AE450\\main_codegen.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(740051.52975694439));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("23.2.0.2409890 (R2023b) Update 3"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)emlrtMexFcnResolvedFunctionsInfo());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("TeISFRScdDW0aEHl0h4SUC"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

//
// File trailer for _coder_main_codegen_info.cpp
//
// [EOF]
//
