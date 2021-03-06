// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: task.proto

#include "task.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
namespace strategy {
class TaskDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Task> _instance;
} _Task_default_instance_;
}  // namespace strategy
static void InitDefaultsscc_info_Task_task_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::strategy::_Task_default_instance_;
    new (ptr) ::strategy::Task();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::strategy::Task::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Task_task_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_Task_task_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_task_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_task_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_task_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_task_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::strategy::Task, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::strategy::Task, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::strategy::Task, action_),
  PROTOBUF_FIELD_OFFSET(::strategy::Task, xloc_),
  PROTOBUF_FIELD_OFFSET(::strategy::Task, yloc_),
  0,
  1,
  2,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::strategy::Task)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::strategy::_Task_default_instance_),
};

const char descriptor_table_protodef_task_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\ntask.proto\022\010strategy\"2\n\004Task\022\016\n\006action"
  "\030\001 \002(\t\022\014\n\004xloc\030\002 \002(\002\022\014\n\004yloc\030\003 \002(\002"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_task_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_task_2eproto_sccs[1] = {
  &scc_info_Task_task_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_task_2eproto_once;
static bool descriptor_table_task_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_task_2eproto = {
  &descriptor_table_task_2eproto_initialized, descriptor_table_protodef_task_2eproto, "task.proto", 74,
  &descriptor_table_task_2eproto_once, descriptor_table_task_2eproto_sccs, descriptor_table_task_2eproto_deps, 1, 0,
  schemas, file_default_instances, TableStruct_task_2eproto::offsets,
  file_level_metadata_task_2eproto, 1, file_level_enum_descriptors_task_2eproto, file_level_service_descriptors_task_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_task_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_task_2eproto), true);
namespace strategy {

// ===================================================================

void Task::InitAsDefaultInstance() {
}
class Task::_Internal {
 public:
  using HasBits = decltype(std::declval<Task>()._has_bits_);
  static void set_has_action(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_xloc(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_yloc(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

Task::Task()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:strategy.Task)
}
Task::Task(const Task& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  action_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_action()) {
    action_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.action_);
  }
  ::memcpy(&xloc_, &from.xloc_,
    static_cast<size_t>(reinterpret_cast<char*>(&yloc_) -
    reinterpret_cast<char*>(&xloc_)) + sizeof(yloc_));
  // @@protoc_insertion_point(copy_constructor:strategy.Task)
}

void Task::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_Task_task_2eproto.base);
  action_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  ::memset(&xloc_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&yloc_) -
      reinterpret_cast<char*>(&xloc_)) + sizeof(yloc_));
}

Task::~Task() {
  // @@protoc_insertion_point(destructor:strategy.Task)
  SharedDtor();
}

void Task::SharedDtor() {
  action_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void Task::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Task& Task::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Task_task_2eproto.base);
  return *internal_default_instance();
}


void Task::Clear() {
// @@protoc_insertion_point(message_clear_start:strategy.Task)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    action_.ClearNonDefaultToEmptyNoArena();
  }
  if (cached_has_bits & 0x00000006u) {
    ::memset(&xloc_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&yloc_) -
        reinterpret_cast<char*>(&xloc_)) + sizeof(yloc_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* Task::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // required string action = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParserUTF8Verify(_internal_mutable_action(), ptr, ctx, "strategy.Task.action");
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // required float xloc = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 21)) {
          _Internal::set_has_xloc(&has_bits);
          xloc_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // required float yloc = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 29)) {
          _Internal::set_has_yloc(&has_bits);
          yloc_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Task::InternalSerializeWithCachedSizesToArray(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:strategy.Task)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required string action = 1;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_action().data(), static_cast<int>(this->_internal_action().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "strategy.Task.action");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_action(), target);
  }

  // required float xloc = 2;
  if (cached_has_bits & 0x00000002u) {
    stream->EnsureSpace(&target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2, this->_internal_xloc(), target);
  }

  // required float yloc = 3;
  if (cached_has_bits & 0x00000004u) {
    stream->EnsureSpace(&target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(3, this->_internal_yloc(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:strategy.Task)
  return target;
}

size_t Task::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:strategy.Task)
  size_t total_size = 0;

  if (has_action()) {
    // required string action = 1;
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_action());
  }

  if (has_xloc()) {
    // required float xloc = 2;
    total_size += 1 + 4;
  }

  if (has_yloc()) {
    // required float yloc = 3;
    total_size += 1 + 4;
  }

  return total_size;
}
size_t Task::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:strategy.Task)
  size_t total_size = 0;

  if (((_has_bits_[0] & 0x00000007) ^ 0x00000007) == 0) {  // All required fields are present.
    // required string action = 1;
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_action());

    // required float xloc = 2;
    total_size += 1 + 4;

    // required float yloc = 3;
    total_size += 1 + 4;

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Task::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:strategy.Task)
  GOOGLE_DCHECK_NE(&from, this);
  const Task* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Task>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:strategy.Task)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:strategy.Task)
    MergeFrom(*source);
  }
}

void Task::MergeFrom(const Task& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:strategy.Task)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _has_bits_[0] |= 0x00000001u;
      action_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.action_);
    }
    if (cached_has_bits & 0x00000002u) {
      xloc_ = from.xloc_;
    }
    if (cached_has_bits & 0x00000004u) {
      yloc_ = from.yloc_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void Task::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:strategy.Task)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Task::CopyFrom(const Task& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:strategy.Task)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Task::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000007) != 0x00000007) return false;
  return true;
}

void Task::InternalSwap(Task* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  action_.Swap(&other->action_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(xloc_, other->xloc_);
  swap(yloc_, other->yloc_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Task::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace strategy
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::strategy::Task* Arena::CreateMaybeMessage< ::strategy::Task >(Arena* arena) {
  return Arena::CreateInternal< ::strategy::Task >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
