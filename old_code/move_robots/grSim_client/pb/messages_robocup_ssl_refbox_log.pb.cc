// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: messages_robocup_ssl_refbox_log.proto

#include "messages_robocup_ssl_refbox_log.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

extern PROTOBUF_INTERNAL_EXPORT_messages_5frobocup_5fssl_5fdetection_2eproto ::google::protobuf::internal::SCCInfo<2> scc_info_SSL_DetectionFrame_messages_5frobocup_5fssl_5fdetection_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_messages_5frobocup_5fssl_5frefbox_5flog_2eproto ::google::protobuf::internal::SCCInfo<1> scc_info_Log_Frame_messages_5frobocup_5fssl_5frefbox_5flog_2eproto;
class Log_FrameDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Log_Frame> _instance;
} _Log_Frame_default_instance_;
class Refbox_LogDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Refbox_Log> _instance;
} _Refbox_Log_default_instance_;
static void InitDefaultsLog_Frame_messages_5frobocup_5fssl_5frefbox_5flog_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::_Log_Frame_default_instance_;
    new (ptr) ::Log_Frame();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::Log_Frame::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_Log_Frame_messages_5frobocup_5fssl_5frefbox_5flog_2eproto =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsLog_Frame_messages_5frobocup_5fssl_5frefbox_5flog_2eproto}, {
      &scc_info_SSL_DetectionFrame_messages_5frobocup_5fssl_5fdetection_2eproto.base,}};

static void InitDefaultsRefbox_Log_messages_5frobocup_5fssl_5frefbox_5flog_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::_Refbox_Log_default_instance_;
    new (ptr) ::Refbox_Log();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::Refbox_Log::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_Refbox_Log_messages_5frobocup_5fssl_5frefbox_5flog_2eproto =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsRefbox_Log_messages_5frobocup_5fssl_5frefbox_5flog_2eproto}, {
      &scc_info_Log_Frame_messages_5frobocup_5fssl_5frefbox_5flog_2eproto.base,}};

void InitDefaults_messages_5frobocup_5fssl_5frefbox_5flog_2eproto() {
  ::google::protobuf::internal::InitSCC(&scc_info_Log_Frame_messages_5frobocup_5fssl_5frefbox_5flog_2eproto.base);
  ::google::protobuf::internal::InitSCC(&scc_info_Refbox_Log_messages_5frobocup_5fssl_5frefbox_5flog_2eproto.base);
}

::google::protobuf::Metadata file_level_metadata_messages_5frobocup_5fssl_5frefbox_5flog_2eproto[2];
constexpr ::google::protobuf::EnumDescriptor const** file_level_enum_descriptors_messages_5frobocup_5fssl_5frefbox_5flog_2eproto = nullptr;
constexpr ::google::protobuf::ServiceDescriptor const** file_level_service_descriptors_messages_5frobocup_5fssl_5frefbox_5flog_2eproto = nullptr;

const ::google::protobuf::uint32 TableStruct_messages_5frobocup_5fssl_5frefbox_5flog_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::Log_Frame, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::Log_Frame, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::Log_Frame, frame_),
  PROTOBUF_FIELD_OFFSET(::Log_Frame, refbox_cmd_),
  1,
  0,
  PROTOBUF_FIELD_OFFSET(::Refbox_Log, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::Refbox_Log, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::Refbox_Log, log_),
  ~0u,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::Log_Frame)},
  { 9, 15, sizeof(::Refbox_Log)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::_Log_Frame_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::_Refbox_Log_default_instance_),
};

::google::protobuf::internal::AssignDescriptorsTable assign_descriptors_table_messages_5frobocup_5fssl_5frefbox_5flog_2eproto = {
  {}, AddDescriptors_messages_5frobocup_5fssl_5frefbox_5flog_2eproto, "messages_robocup_ssl_refbox_log.proto", schemas,
  file_default_instances, TableStruct_messages_5frobocup_5fssl_5frefbox_5flog_2eproto::offsets,
  file_level_metadata_messages_5frobocup_5fssl_5frefbox_5flog_2eproto, 2, file_level_enum_descriptors_messages_5frobocup_5fssl_5frefbox_5flog_2eproto, file_level_service_descriptors_messages_5frobocup_5fssl_5frefbox_5flog_2eproto,
};

const char descriptor_table_protodef_messages_5frobocup_5fssl_5frefbox_5flog_2eproto[] =
  "\n%messages_robocup_ssl_refbox_log.proto\032"
  "$messages_robocup_ssl_detection.proto\"C\n"
  "\tLog_Frame\022\"\n\005frame\030\001 \002(\0132\023.SSL_Detectio"
  "nFrame\022\022\n\nrefbox_cmd\030\002 \002(\t\"%\n\nRefbox_Log"
  "\022\027\n\003log\030\001 \003(\0132\n.Log_Frame"
  ;
::google::protobuf::internal::DescriptorTable descriptor_table_messages_5frobocup_5fssl_5frefbox_5flog_2eproto = {
  false, InitDefaults_messages_5frobocup_5fssl_5frefbox_5flog_2eproto, 
  descriptor_table_protodef_messages_5frobocup_5fssl_5frefbox_5flog_2eproto,
  "messages_robocup_ssl_refbox_log.proto", &assign_descriptors_table_messages_5frobocup_5fssl_5frefbox_5flog_2eproto, 185,
};

void AddDescriptors_messages_5frobocup_5fssl_5frefbox_5flog_2eproto() {
  static constexpr ::google::protobuf::internal::InitFunc deps[1] =
  {
    ::AddDescriptors_messages_5frobocup_5fssl_5fdetection_2eproto,
  };
 ::google::protobuf::internal::AddDescriptors(&descriptor_table_messages_5frobocup_5fssl_5frefbox_5flog_2eproto, deps, 1);
}

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_messages_5frobocup_5fssl_5frefbox_5flog_2eproto = []() { AddDescriptors_messages_5frobocup_5fssl_5frefbox_5flog_2eproto(); return true; }();

// ===================================================================

void Log_Frame::InitAsDefaultInstance() {
  ::_Log_Frame_default_instance_._instance.get_mutable()->frame_ = const_cast< ::SSL_DetectionFrame*>(
      ::SSL_DetectionFrame::internal_default_instance());
}
class Log_Frame::HasBitSetters {
 public:
  static const ::SSL_DetectionFrame& frame(const Log_Frame* msg);
  static void set_has_frame(Log_Frame* msg) {
    msg->_has_bits_[0] |= 0x00000002u;
  }
  static void set_has_refbox_cmd(Log_Frame* msg) {
    msg->_has_bits_[0] |= 0x00000001u;
  }
};

const ::SSL_DetectionFrame&
Log_Frame::HasBitSetters::frame(const Log_Frame* msg) {
  return *msg->frame_;
}
void Log_Frame::clear_frame() {
  if (frame_ != nullptr) frame_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Log_Frame::kFrameFieldNumber;
const int Log_Frame::kRefboxCmdFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Log_Frame::Log_Frame()
  : ::google::protobuf::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:Log_Frame)
}
Log_Frame::Log_Frame(const Log_Frame& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  refbox_cmd_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_refbox_cmd()) {
    refbox_cmd_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.refbox_cmd_);
  }
  if (from.has_frame()) {
    frame_ = new ::SSL_DetectionFrame(*from.frame_);
  } else {
    frame_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:Log_Frame)
}

void Log_Frame::SharedCtor() {
  ::google::protobuf::internal::InitSCC(
      &scc_info_Log_Frame_messages_5frobocup_5fssl_5frefbox_5flog_2eproto.base);
  refbox_cmd_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  frame_ = nullptr;
}

Log_Frame::~Log_Frame() {
  // @@protoc_insertion_point(destructor:Log_Frame)
  SharedDtor();
}

void Log_Frame::SharedDtor() {
  refbox_cmd_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete frame_;
}

void Log_Frame::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Log_Frame& Log_Frame::default_instance() {
  ::google::protobuf::internal::InitSCC(&::scc_info_Log_Frame_messages_5frobocup_5fssl_5frefbox_5flog_2eproto.base);
  return *internal_default_instance();
}


void Log_Frame::Clear() {
// @@protoc_insertion_point(message_clear_start:Log_Frame)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      refbox_cmd_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(frame_ != nullptr);
      frame_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* Log_Frame::_InternalParse(const char* begin, const char* end, void* object,
                  ::google::protobuf::internal::ParseContext* ctx) {
  auto msg = static_cast<Log_Frame*>(object);
  ::google::protobuf::int32 size; (void)size;
  int depth; (void)depth;
  ::google::protobuf::uint32 tag;
  ::google::protobuf::internal::ParseFunc parser_till_end; (void)parser_till_end;
  auto ptr = begin;
  while (ptr < end) {
    ptr = ::google::protobuf::io::Parse32(ptr, &tag);
    GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
    switch (tag >> 3) {
      // required .SSL_DetectionFrame frame = 1;
      case 1: {
        if (static_cast<::google::protobuf::uint8>(tag) != 10) goto handle_unusual;
        ptr = ::google::protobuf::io::ReadSize(ptr, &size);
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
        parser_till_end = ::SSL_DetectionFrame::_InternalParse;
        object = msg->mutable_frame();
        if (size > end - ptr) goto len_delim_till_end;
        ptr += size;
        GOOGLE_PROTOBUF_PARSER_ASSERT(ctx->ParseExactRange(
            {parser_till_end, object}, ptr - size, ptr));
        break;
      }
      // required string refbox_cmd = 2;
      case 2: {
        if (static_cast<::google::protobuf::uint8>(tag) != 18) goto handle_unusual;
        ptr = ::google::protobuf::io::ReadSize(ptr, &size);
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
        ctx->extra_parse_data().SetFieldName("Log_Frame.refbox_cmd");
        object = msg->mutable_refbox_cmd();
        if (size > end - ptr + ::google::protobuf::internal::ParseContext::kSlopBytes) {
          parser_till_end = ::google::protobuf::internal::GreedyStringParserUTF8Verify;
          goto string_till_end;
        }
        GOOGLE_PROTOBUF_PARSER_ASSERT(::google::protobuf::internal::StringCheckUTF8Verify(ptr, size, ctx));
        ::google::protobuf::internal::InlineGreedyStringParser(object, ptr, size, ctx);
        ptr += size;
        break;
      }
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->EndGroup(tag);
          return ptr;
        }
        auto res = UnknownFieldParse(tag, {_InternalParse, msg},
          ptr, end, msg->_internal_metadata_.mutable_unknown_fields(), ctx);
        ptr = res.first;
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr != nullptr);
        if (res.second) return ptr;
      }
    }  // switch
  }  // while
  return ptr;
string_till_end:
  static_cast<::std::string*>(object)->clear();
  static_cast<::std::string*>(object)->reserve(size);
  goto len_delim_till_end;
len_delim_till_end:
  return ctx->StoreAndTailCall(ptr, end, {_InternalParse, msg},
                               {parser_till_end, object}, size);
}
#else  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
bool Log_Frame::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:Log_Frame)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required .SSL_DetectionFrame frame = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (10 & 0xFF)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_frame()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required string refbox_cmd = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (18 & 0xFF)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_refbox_cmd()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->refbox_cmd().data(), static_cast<int>(this->refbox_cmd().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "Log_Frame.refbox_cmd");
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:Log_Frame)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:Log_Frame)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void Log_Frame::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:Log_Frame)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required .SSL_DetectionFrame frame = 1;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, HasBitSetters::frame(this), output);
  }

  // required string refbox_cmd = 2;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->refbox_cmd().data(), static_cast<int>(this->refbox_cmd().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "Log_Frame.refbox_cmd");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      2, this->refbox_cmd(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:Log_Frame)
}

::google::protobuf::uint8* Log_Frame::InternalSerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:Log_Frame)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required .SSL_DetectionFrame frame = 1;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, HasBitSetters::frame(this), target);
  }

  // required string refbox_cmd = 2;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->refbox_cmd().data(), static_cast<int>(this->refbox_cmd().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "Log_Frame.refbox_cmd");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        2, this->refbox_cmd(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:Log_Frame)
  return target;
}

size_t Log_Frame::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:Log_Frame)
  size_t total_size = 0;

  if (has_refbox_cmd()) {
    // required string refbox_cmd = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->refbox_cmd());
  }

  if (has_frame()) {
    // required .SSL_DetectionFrame frame = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *frame_);
  }

  return total_size;
}
size_t Log_Frame::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:Log_Frame)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (((_has_bits_[0] & 0x00000003) ^ 0x00000003) == 0) {  // All required fields are present.
    // required string refbox_cmd = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->refbox_cmd());

    // required .SSL_DetectionFrame frame = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *frame_);

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Log_Frame::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:Log_Frame)
  GOOGLE_DCHECK_NE(&from, this);
  const Log_Frame* source =
      ::google::protobuf::DynamicCastToGenerated<Log_Frame>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:Log_Frame)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:Log_Frame)
    MergeFrom(*source);
  }
}

void Log_Frame::MergeFrom(const Log_Frame& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:Log_Frame)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      _has_bits_[0] |= 0x00000001u;
      refbox_cmd_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.refbox_cmd_);
    }
    if (cached_has_bits & 0x00000002u) {
      mutable_frame()->::SSL_DetectionFrame::MergeFrom(from.frame());
    }
  }
}

void Log_Frame::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:Log_Frame)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Log_Frame::CopyFrom(const Log_Frame& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:Log_Frame)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Log_Frame::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;
  if (has_frame()) {
    if (!this->frame_->IsInitialized()) return false;
  }
  return true;
}

void Log_Frame::Swap(Log_Frame* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Log_Frame::InternalSwap(Log_Frame* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  refbox_cmd_.Swap(&other->refbox_cmd_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(frame_, other->frame_);
}

::google::protobuf::Metadata Log_Frame::GetMetadata() const {
  ::google::protobuf::internal::AssignDescriptors(&::assign_descriptors_table_messages_5frobocup_5fssl_5frefbox_5flog_2eproto);
  return ::file_level_metadata_messages_5frobocup_5fssl_5frefbox_5flog_2eproto[kIndexInFileMessages];
}


// ===================================================================

void Refbox_Log::InitAsDefaultInstance() {
}
class Refbox_Log::HasBitSetters {
 public:
};

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Refbox_Log::kLogFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Refbox_Log::Refbox_Log()
  : ::google::protobuf::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:Refbox_Log)
}
Refbox_Log::Refbox_Log(const Refbox_Log& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      log_(from.log_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:Refbox_Log)
}

void Refbox_Log::SharedCtor() {
  ::google::protobuf::internal::InitSCC(
      &scc_info_Refbox_Log_messages_5frobocup_5fssl_5frefbox_5flog_2eproto.base);
}

Refbox_Log::~Refbox_Log() {
  // @@protoc_insertion_point(destructor:Refbox_Log)
  SharedDtor();
}

void Refbox_Log::SharedDtor() {
}

void Refbox_Log::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Refbox_Log& Refbox_Log::default_instance() {
  ::google::protobuf::internal::InitSCC(&::scc_info_Refbox_Log_messages_5frobocup_5fssl_5frefbox_5flog_2eproto.base);
  return *internal_default_instance();
}


void Refbox_Log::Clear() {
// @@protoc_insertion_point(message_clear_start:Refbox_Log)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  log_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* Refbox_Log::_InternalParse(const char* begin, const char* end, void* object,
                  ::google::protobuf::internal::ParseContext* ctx) {
  auto msg = static_cast<Refbox_Log*>(object);
  ::google::protobuf::int32 size; (void)size;
  int depth; (void)depth;
  ::google::protobuf::uint32 tag;
  ::google::protobuf::internal::ParseFunc parser_till_end; (void)parser_till_end;
  auto ptr = begin;
  while (ptr < end) {
    ptr = ::google::protobuf::io::Parse32(ptr, &tag);
    GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
    switch (tag >> 3) {
      // repeated .Log_Frame log = 1;
      case 1: {
        if (static_cast<::google::protobuf::uint8>(tag) != 10) goto handle_unusual;
        do {
          ptr = ::google::protobuf::io::ReadSize(ptr, &size);
          GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
          parser_till_end = ::Log_Frame::_InternalParse;
          object = msg->add_log();
          if (size > end - ptr) goto len_delim_till_end;
          ptr += size;
          GOOGLE_PROTOBUF_PARSER_ASSERT(ctx->ParseExactRange(
              {parser_till_end, object}, ptr - size, ptr));
          if (ptr >= end) break;
        } while ((::google::protobuf::io::UnalignedLoad<::google::protobuf::uint64>(ptr) & 255) == 10 && (ptr += 1));
        break;
      }
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->EndGroup(tag);
          return ptr;
        }
        auto res = UnknownFieldParse(tag, {_InternalParse, msg},
          ptr, end, msg->_internal_metadata_.mutable_unknown_fields(), ctx);
        ptr = res.first;
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr != nullptr);
        if (res.second) return ptr;
      }
    }  // switch
  }  // while
  return ptr;
len_delim_till_end:
  return ctx->StoreAndTailCall(ptr, end, {_InternalParse, msg},
                               {parser_till_end, object}, size);
}
#else  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
bool Refbox_Log::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:Refbox_Log)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .Log_Frame log = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (10 & 0xFF)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_log()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:Refbox_Log)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:Refbox_Log)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void Refbox_Log::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:Refbox_Log)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .Log_Frame log = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->log_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1,
      this->log(static_cast<int>(i)),
      output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:Refbox_Log)
}

::google::protobuf::uint8* Refbox_Log::InternalSerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:Refbox_Log)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .Log_Frame log = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->log_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->log(static_cast<int>(i)), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:Refbox_Log)
  return target;
}

size_t Refbox_Log::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:Refbox_Log)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .Log_Frame log = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->log_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->log(static_cast<int>(i)));
    }
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Refbox_Log::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:Refbox_Log)
  GOOGLE_DCHECK_NE(&from, this);
  const Refbox_Log* source =
      ::google::protobuf::DynamicCastToGenerated<Refbox_Log>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:Refbox_Log)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:Refbox_Log)
    MergeFrom(*source);
  }
}

void Refbox_Log::MergeFrom(const Refbox_Log& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:Refbox_Log)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  log_.MergeFrom(from.log_);
}

void Refbox_Log::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:Refbox_Log)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Refbox_Log::CopyFrom(const Refbox_Log& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:Refbox_Log)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Refbox_Log::IsInitialized() const {
  if (!::google::protobuf::internal::AllAreInitialized(this->log())) return false;
  return true;
}

void Refbox_Log::Swap(Refbox_Log* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Refbox_Log::InternalSwap(Refbox_Log* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  CastToBase(&log_)->InternalSwap(CastToBase(&other->log_));
}

::google::protobuf::Metadata Refbox_Log::GetMetadata() const {
  ::google::protobuf::internal::AssignDescriptors(&::assign_descriptors_table_messages_5frobocup_5fssl_5frefbox_5flog_2eproto);
  return ::file_level_metadata_messages_5frobocup_5fssl_5frefbox_5flog_2eproto[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
namespace google {
namespace protobuf {
template<> PROTOBUF_NOINLINE ::Log_Frame* Arena::CreateMaybeMessage< ::Log_Frame >(Arena* arena) {
  return Arena::CreateInternal< ::Log_Frame >(arena);
}
template<> PROTOBUF_NOINLINE ::Refbox_Log* Arena::CreateMaybeMessage< ::Refbox_Log >(Arena* arena) {
  return Arena::CreateInternal< ::Refbox_Log >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
