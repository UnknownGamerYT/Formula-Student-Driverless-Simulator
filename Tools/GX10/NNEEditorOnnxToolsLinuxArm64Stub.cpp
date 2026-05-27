// Minimal LinuxArm64 replacement for UE's editor-only NNEEditorOnnxTools helper.
// Epic ships this helper as a binary-only x86_64 Linux library in UE 5.5.

#include <cstdint>
#include <new>

enum class NNEEditorOnnxTools_Status : std::uint8_t
{
	Ok = 0,
	Fail_CannotParseAsModelProto
};

struct NNEEditorOnnxTools_ExternalDataDescriptor
{
	bool bConsumed = false;
};

extern "C" __attribute__((visibility("default"))) NNEEditorOnnxTools_Status
NNEEditorOnnxTools_CreateExternalDataDescriptor(
	const void* InData,
	const int Size,
	NNEEditorOnnxTools_ExternalDataDescriptor** Descriptor)
{
	if (InData == nullptr || Size < 0 || Descriptor == nullptr)
	{
		return NNEEditorOnnxTools_Status::Fail_CannotParseAsModelProto;
	}

	*Descriptor = new (std::nothrow) NNEEditorOnnxTools_ExternalDataDescriptor();
	return *Descriptor != nullptr
		? NNEEditorOnnxTools_Status::Ok
		: NNEEditorOnnxTools_Status::Fail_CannotParseAsModelProto;
}

extern "C" __attribute__((visibility("default"))) void
NNEEditorOnnxTools_ReleaseExternalDataDescriptor(
	NNEEditorOnnxTools_ExternalDataDescriptor** Descriptor)
{
	if (Descriptor != nullptr)
	{
		delete *Descriptor;
		*Descriptor = nullptr;
	}
}

extern "C" __attribute__((visibility("default"))) const char*
NNEEditorOnnxTools_GetNextExternalDataPath(
	NNEEditorOnnxTools_ExternalDataDescriptor* Descriptor)
{
	if (Descriptor == nullptr || Descriptor->bConsumed)
	{
		return nullptr;
	}

	Descriptor->bConsumed = true;
	return nullptr;
}
