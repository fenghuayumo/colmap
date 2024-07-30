project "colmap_feature"
    kind "StaticLib"
	language "C++"
	-- editandcontinue "Off"

	files
	{
		"**.h",
		"**.c",
		"**.cc",
        -- "src/**.cu"
	}

	includedirs
	{
		"../../",
	}

	externalincludedirs
	{
		"%{IncludeDir.Package}",
	}

    libdirs
    {
        "%{LibraryDir.Package}",
    }

    links
	{
        "colmap_util",
        "colmap_math",
        "colmap_sensor",
        "flann",
        "lz4"
    }

	defines
	{
        "GLOG_USE_GLOG_EXPORT",
        "GLOG_VERSION_MAJOR 1",
        "COLMAP_CUDA_ENABLED"
    }

    buildcustomizations {cudaBuildCustomizations}

    if os.target() == "windows" then
      -- Just in case we want the VS CUDA extension to use a custom version of CUDA
      cudaPath "$(CUDA_PATH)"
    else
      toolset "nvcc"
      cudaPath "/usr/local/cuda"
      rules {"cu"}
    end
   
    cudaRelocatableCode "On"
   
    -- Let's compile for all supported architectures (and also in parallel with -t0)
cudaCompilerOptions {"--generate-code=arch=compute_60,code=[compute_60,sm_60] --generate-code=arch=compute_70,code=[compute_70,sm_70] --generate-code=arch=compute_80,code=[compute_80,sm_80]", "-t0", "-std=c++17"} 
    cudaFastMath "On"
    cudaIntDir "bin/cudaobj/%{cfg.buildcfg}"
	
	filter "system:windows"
		cppdialect "C++20"
		systemversion "latest"
		disablewarnings { 4307 }
		characterset ("Unicode")
		conformancemode "on"
        externalincludedirs
        {
            "%{IncludeDir.Package}"
        }
        links
        {
            
        }

    filter "configurations:Debug"
        defines { "_DEBUG" }
        symbols "On"
        runtime "Debug"
        optimize "Off"
    
    filter "configurations:Release"
        defines {"NDEBUG" }
        optimize "Speed"
        symbols "On"
        runtime "Release"
    
    filter "configurations:Production"
        defines { "NDEBUG" }
        symbols "Off"
        optimize "Full"
        runtime "Release"
    
