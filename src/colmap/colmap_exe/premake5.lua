project "colmap_exe"
	kind "ConsoleApp"
	language "C++"
	-- editandcontinue "Off"

	files
	{
		"**.h",
		"**.c",
		"**.cpp",
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
        -- "glog",
        -- "boost_filesystem-vc140-mt",
        -- "boost_program_options-vc140-mt",
        -- "gtest",
        -- "colmap_controllers",
        -- "colmap_estimators",
        -- "colmap_util",
        -- "colmap_geometry",
        -- "colmap_sfm",
        -- "colmap_optim",
        -- "colmap_scene",
        -- "colmap_image",
        -- "colmap_math",
        -- "colmap_mvs",
        -- "ceres",
        -- "FreeImage",
        -- "colmap_sift_gpu",
        -- "colmap_vlfeat",
        -- "sqlite3"
        "colmap"
    }

	defines
	{
        "GLOG_USE_GLOG_EXPORT",
        "GLOG_VERSION_MAJOR 1",
        "COlMAP_DYNAMIC",
    }
	
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
    
