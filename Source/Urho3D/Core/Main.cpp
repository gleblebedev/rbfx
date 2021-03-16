#include "Main.h"
#include "Context.h"

typedef int(*sdl_callback)(Urho3D::Context *);

sdl_callback sdlCallback = nullptr;
const char *sdlResourceDir = nullptr;
const char *sdlDocumentsDir = nullptr;

extern "C"
{
	void URHO3D_API RegisterSdlLauncher(sdl_callback callback)
	{
		sdlCallback = callback;
	}

	void URHO3D_API InitSdl(const char *resourceDir, const char *documentsDir)
	{
		sdlResourceDir = resourceDir;
		sdlDocumentsDir = documentsDir;
	}
}

//FileSystem.cpp uses these functions as external.
#if defined(IOS)
const char* SDL_IOS_GetResourceDir()
{
	return sdlResourceDir;
}

const char* SDL_IOS_GetDocumentsDir()
{
	return sdlDocumentsDir;
}
#endif

#if defined(ANDROID) || defined(IOS)
// Entry point for SDL (Android or iOS)
int RunApplication()
{
    if (sdlCallback)
	    return sdlCallback(NULL);
    return -1;
}
URHO3D_DEFINE_MAIN(RunApplication());
#endif
