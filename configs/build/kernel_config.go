// Copyright (C) 2018 The Android Open Source Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package configs

import (
	"strings"

	"github.com/google/blueprint"
	"github.com/google/blueprint/proptools"

	"android/soong/android"
)

var (
	pctx = android.NewPackageContext("android/soong/kernel/configs")

	kconfigXmlFixupRule = pctx.AndroidStaticRule("kconfig_xml_fixup", blueprint.RuleParams{
		Command:     `${kconfigXmlFixupCmd} --input ${in} --output-version ${outputVersion} --output-matrix ${out}`,
		CommandDeps:  []string{"${kconfigXmlFixupCmd}"},
		Description: "kconfig_xml_fixup ${in}",
	}, "outputVersion")

	assembleVintfRule = pctx.AndroidStaticRule("assemble_vintf", blueprint.RuleParams{
		Command:     `${assembleVintfCmd} ${flags} -i ${in} -o ${out}`,
		CommandDeps:  []string{"${assembleVintfCmd}"},
		Description: "assemble_vintf -i ${in}",
	}, "flags")
)

type KernelConfigProperties struct {
	// List of source files. There should be:
	// 1. Exactly One "android-base.config" as common requirements.
	// 2. Zero or more file with names that does NOT start with "android-base-" as extra common requirements.
	// 3. Zero or more "android-base-foo.config" for requirements on condition CONFIG_FOO=y.
	//    (deprecated; use Meta to express conditional requirements instead.)
	Srcs []string

	// metadata XML file that contains minlts and complex conditional requirements.
	Meta *string

	// list of source files to replace Srcs on debuggable builds. See docs for Srcs.
	Debuggable_srcs []string
}

type KernelConfigRule struct {
	android.ModuleBase
	properties KernelConfigProperties

	outputPath android.WritablePath
}

func init() {
	pctx.HostBinToolVariable("assembleVintfCmd", "assemble_vintf")
	pctx.HostBinToolVariable("kconfigXmlFixupCmd", "kconfig_xml_fixup")
	android.RegisterModuleType("kernel_config", kernelConfigFactory)
}

func kernelConfigFactory() android.Module {
	g := &KernelConfigRule{}
	g.AddProperties(&g.properties)
	android.InitAndroidModule(g)
	return g
}

func (g *KernelConfigRule) OutputPath() android.Path {
	return g.outputPath
}

func (g *KernelConfigRule) realSrcs(ctx android.BaseModuleContext) []string {
	if ctx.Config().Debuggable() && len(g.properties.Debuggable_srcs) > 0 {
		return g.properties.Debuggable_srcs
	} else {
		return g.properties.Srcs
	}
}

func (g *KernelConfigRule) DepsMutator(ctx android.BottomUpMutatorContext) {
	android.ExtractSourcesDeps(ctx, g.realSrcs(ctx))
	android.ExtractSourceDeps(ctx, g.properties.Meta)
}

func (g *KernelConfigRule) GenerateAndroidBuildActions(ctx android.ModuleContext) {
	g.outputPath = android.PathForModuleOut(ctx, "matrix_" + g.Name() + ".xml")
	genVersion := android.PathForModuleGen(ctx, "version.txt")
	genConditionals := android.PathForModuleGen(ctx, "conditional.xml")
	inputMeta := android.PathForModuleSrc(ctx, proptools.String(g.properties.Meta))

	if proptools.String(g.properties.Meta) == "" {
		ctx.PropertyErrorf("kernel_config", "Missing meta field")
	}

	ctx.Build(pctx, android.BuildParams{
		Rule:           kconfigXmlFixupRule,
		Description:    "Fixup kernel config meta",
		Input:          inputMeta,
		Output:         genConditionals,
		ImplicitOutput: genVersion,
		Args: map[string]string{
			"outputVersion": genVersion.String(),
		},
	})

	var kernelArg string
	inputConfigs := android.PathsForModuleSrc(ctx, g.realSrcs(ctx))
	implicitInputs := append(inputConfigs, genVersion)
	if len(inputConfigs) > 0 {
		kernelArg = "--kernel=$$(cat " + genVersion.String() + "):" +
			strings.Join(inputConfigs.Strings(), ":")
	}

	ctx.Build(pctx, android.BuildParams{
		Rule:        assembleVintfRule,
		Description: "Framework Compatibility Matrix kernel fragment",
		Input:       genConditionals,
		Implicits:   implicitInputs,
		Output:      g.outputPath,
		Args: map[string]string{
			"flags": kernelArg,
		},
	})

}
