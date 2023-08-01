project "Physix2D"
	kind "StaticLib"
	language "C++"
	cppdialect "C++17"
	staticruntime "on"
	
	targetdir (outputdir .. "/%{prj.name}")
	objdir (intdir .. "/%{prj.name}")

	files
	{
		"src/**.h",
		"src/**.cpp",
	}

	includedirs
	{
		"src",
	}

	filter "system:windows"
		systemversion "latest"
		
	filter "configurations:Debug"
		defines "PHX_DEBUG"
		runtime "Debug"
		symbols "on"

	filter "configurations:Release"
		defines "PHX_RELEASE"	
		runtime "Release"
		optimize "on"

	filter "configurations:Dist"
		defines "PHX_DIST"
		runtime "Release"
		optimize "on"