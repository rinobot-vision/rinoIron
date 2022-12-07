// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vssref_common.proto

#include "vssref_common.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace protobuf_vssref_5fcommon_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_vssref_5fcommon_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Robot;
}  // namespace protobuf_vssref_5fcommon_2eproto
namespace VSSRef {
class RobotDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Robot>
      _instance;
} _Robot_default_instance_;
class FrameDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Frame>
      _instance;
} _Frame_default_instance_;
}  // namespace VSSRef
namespace protobuf_vssref_5fcommon_2eproto {
static void InitDefaultsRobot() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::VSSRef::_Robot_default_instance_;
    new (ptr) ::VSSRef::Robot();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::VSSRef::Robot::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_Robot =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsRobot}, {}};

static void InitDefaultsFrame() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::VSSRef::_Frame_default_instance_;
    new (ptr) ::VSSRef::Frame();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::VSSRef::Frame::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_Frame =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsFrame}, {
      &protobuf_vssref_5fcommon_2eproto::scc_info_Robot.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_Robot.base);
  ::google::protobuf::internal::InitSCC(&scc_info_Frame.base);
}

::google::protobuf::Metadata file_level_metadata[2];
const ::google::protobuf::EnumDescriptor* file_level_enum_descriptors[4];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::VSSRef::Robot, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::VSSRef::Robot, robot_id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::VSSRef::Robot, x_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::VSSRef::Robot, y_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::VSSRef::Robot, orientation_),
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::VSSRef::Frame, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::VSSRef::Frame, teamcolor_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::VSSRef::Frame, robots_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::VSSRef::Robot)},
  { 9, -1, sizeof(::VSSRef::Frame)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::VSSRef::_Robot_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::VSSRef::_Frame_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "vssref_common.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, file_level_enum_descriptors, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 2);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\023vssref_common.proto\022\006VSSRef\"D\n\005Robot\022\020"
      "\n\010robot_id\030\001 \001(\r\022\t\n\001x\030\002 \001(\001\022\t\n\001y\030\003 \001(\001\022\023"
      "\n\013orientation\030\004 \001(\001\"H\n\005Frame\022 \n\tteamColo"
      "r\030\001 \001(\0162\r.VSSRef.Color\022\035\n\006robots\030\002 \003(\0132\r"
      ".VSSRef.Robot*s\n\004Foul\022\r\n\tFREE_KICK\020\000\022\020\n\014"
      "PENALTY_KICK\020\001\022\r\n\tGOAL_KICK\020\002\022\r\n\tFREE_BA"
      "LL\020\003\022\013\n\007KICKOFF\020\004\022\010\n\004STOP\020\005\022\013\n\007GAME_ON\020\006"
      "\022\010\n\004HALT\020\007*\'\n\005Color\022\010\n\004BLUE\020\000\022\n\n\006YELLOW\020"
      "\001\022\010\n\004NONE\020\002*[\n\010Quadrant\022\017\n\013NO_QUADRANT\020\000"
      "\022\016\n\nQUADRANT_1\020\001\022\016\n\nQUADRANT_2\020\002\022\016\n\nQUAD"
      "RANT_3\020\003\022\016\n\nQUADRANT_4\020\004*~\n\004Half\022\013\n\007NO_H"
      "ALF\020\000\022\016\n\nFIRST_HALF\020\001\022\017\n\013SECOND_HALF\020\002\022\027"
      "\n\023OVERTIME_FIRST_HALF\020\003\022\030\n\024OVERTIME_SECO"
      "ND_HALF\020\004\022\025\n\021PENALTY_SHOOTOUTS\020\005b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 560);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "vssref_common.proto", &protobuf_RegisterTypes);
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_vssref_5fcommon_2eproto
namespace VSSRef {
const ::google::protobuf::EnumDescriptor* Foul_descriptor() {
  protobuf_vssref_5fcommon_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_vssref_5fcommon_2eproto::file_level_enum_descriptors[0];
}
bool Foul_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
      return true;
    default:
      return false;
  }
}

const ::google::protobuf::EnumDescriptor* Color_descriptor() {
  protobuf_vssref_5fcommon_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_vssref_5fcommon_2eproto::file_level_enum_descriptors[1];
}
bool Color_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}

const ::google::protobuf::EnumDescriptor* Quadrant_descriptor() {
  protobuf_vssref_5fcommon_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_vssref_5fcommon_2eproto::file_level_enum_descriptors[2];
}
bool Quadrant_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
      return true;
    default:
      return false;
  }
}

const ::google::protobuf::EnumDescriptor* Half_descriptor() {
  protobuf_vssref_5fcommon_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_vssref_5fcommon_2eproto::file_level_enum_descriptors[3];
}
bool Half_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
      return true;
    default:
      return false;
  }
}


// ===================================================================

void Robot::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Robot::kRobotIdFieldNumber;
const int Robot::kXFieldNumber;
const int Robot::kYFieldNumber;
const int Robot::kOrientationFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Robot::Robot()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_vssref_5fcommon_2eproto::scc_info_Robot.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:VSSRef.Robot)
}
Robot::Robot(const Robot& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&x_, &from.x_,
    static_cast<size_t>(reinterpret_cast<char*>(&robot_id_) -
    reinterpret_cast<char*>(&x_)) + sizeof(robot_id_));
  // @@protoc_insertion_point(copy_constructor:VSSRef.Robot)
}

void Robot::SharedCtor() {
  ::memset(&x_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&robot_id_) -
      reinterpret_cast<char*>(&x_)) + sizeof(robot_id_));
}

Robot::~Robot() {
  // @@protoc_insertion_point(destructor:VSSRef.Robot)
  SharedDtor();
}

void Robot::SharedDtor() {
}

void Robot::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* Robot::descriptor() {
  ::protobuf_vssref_5fcommon_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_vssref_5fcommon_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Robot& Robot::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_vssref_5fcommon_2eproto::scc_info_Robot.base);
  return *internal_default_instance();
}


void Robot::Clear() {
// @@protoc_insertion_point(message_clear_start:VSSRef.Robot)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&x_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&robot_id_) -
      reinterpret_cast<char*>(&x_)) + sizeof(robot_id_));
  _internal_metadata_.Clear();
}

bool Robot::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:VSSRef.Robot)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // uint32 robot_id = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &robot_id_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double x = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &x_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double y = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(25u /* 25 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &y_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double orientation = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(33u /* 33 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &orientation_)));
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
  // @@protoc_insertion_point(parse_success:VSSRef.Robot)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:VSSRef.Robot)
  return false;
#undef DO_
}

void Robot::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:VSSRef.Robot)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // uint32 robot_id = 1;
  if (this->robot_id() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(1, this->robot_id(), output);
  }

  // double x = 2;
  if (this->x() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->x(), output);
  }

  // double y = 3;
  if (this->y() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->y(), output);
  }

  // double orientation = 4;
  if (this->orientation() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->orientation(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:VSSRef.Robot)
}

::google::protobuf::uint8* Robot::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:VSSRef.Robot)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // uint32 robot_id = 1;
  if (this->robot_id() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(1, this->robot_id(), target);
  }

  // double x = 2;
  if (this->x() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->x(), target);
  }

  // double y = 3;
  if (this->y() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->y(), target);
  }

  // double orientation = 4;
  if (this->orientation() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->orientation(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:VSSRef.Robot)
  return target;
}

size_t Robot::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:VSSRef.Robot)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // double x = 2;
  if (this->x() != 0) {
    total_size += 1 + 8;
  }

  // double y = 3;
  if (this->y() != 0) {
    total_size += 1 + 8;
  }

  // double orientation = 4;
  if (this->orientation() != 0) {
    total_size += 1 + 8;
  }

  // uint32 robot_id = 1;
  if (this->robot_id() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->robot_id());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Robot::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:VSSRef.Robot)
  GOOGLE_DCHECK_NE(&from, this);
  const Robot* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Robot>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:VSSRef.Robot)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:VSSRef.Robot)
    MergeFrom(*source);
  }
}

void Robot::MergeFrom(const Robot& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:VSSRef.Robot)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.x() != 0) {
    set_x(from.x());
  }
  if (from.y() != 0) {
    set_y(from.y());
  }
  if (from.orientation() != 0) {
    set_orientation(from.orientation());
  }
  if (from.robot_id() != 0) {
    set_robot_id(from.robot_id());
  }
}

void Robot::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:VSSRef.Robot)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Robot::CopyFrom(const Robot& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:VSSRef.Robot)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Robot::IsInitialized() const {
  return true;
}

void Robot::Swap(Robot* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Robot::InternalSwap(Robot* other) {
  using std::swap;
  swap(x_, other->x_);
  swap(y_, other->y_);
  swap(orientation_, other->orientation_);
  swap(robot_id_, other->robot_id_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Robot::GetMetadata() const {
  protobuf_vssref_5fcommon_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_vssref_5fcommon_2eproto::file_level_metadata[kIndexInFileMessages];
}


// ===================================================================

void Frame::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Frame::kTeamColorFieldNumber;
const int Frame::kRobotsFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Frame::Frame()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_vssref_5fcommon_2eproto::scc_info_Frame.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:VSSRef.Frame)
}
Frame::Frame(const Frame& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      robots_(from.robots_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  teamcolor_ = from.teamcolor_;
  // @@protoc_insertion_point(copy_constructor:VSSRef.Frame)
}

void Frame::SharedCtor() {
  teamcolor_ = 0;
}

Frame::~Frame() {
  // @@protoc_insertion_point(destructor:VSSRef.Frame)
  SharedDtor();
}

void Frame::SharedDtor() {
}

void Frame::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* Frame::descriptor() {
  ::protobuf_vssref_5fcommon_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_vssref_5fcommon_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Frame& Frame::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_vssref_5fcommon_2eproto::scc_info_Frame.base);
  return *internal_default_instance();
}


void Frame::Clear() {
// @@protoc_insertion_point(message_clear_start:VSSRef.Frame)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  robots_.Clear();
  teamcolor_ = 0;
  _internal_metadata_.Clear();
}

bool Frame::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:VSSRef.Frame)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // .VSSRef.Color teamColor = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          set_teamcolor(static_cast< ::VSSRef::Color >(value));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .VSSRef.Robot robots = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_robots()));
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
  // @@protoc_insertion_point(parse_success:VSSRef.Frame)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:VSSRef.Frame)
  return false;
#undef DO_
}

void Frame::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:VSSRef.Frame)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .VSSRef.Color teamColor = 1;
  if (this->teamcolor() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      1, this->teamcolor(), output);
  }

  // repeated .VSSRef.Robot robots = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->robots_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2,
      this->robots(static_cast<int>(i)),
      output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:VSSRef.Frame)
}

::google::protobuf::uint8* Frame::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:VSSRef.Frame)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .VSSRef.Color teamColor = 1;
  if (this->teamcolor() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      1, this->teamcolor(), target);
  }

  // repeated .VSSRef.Robot robots = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->robots_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, this->robots(static_cast<int>(i)), deterministic, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:VSSRef.Frame)
  return target;
}

size_t Frame::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:VSSRef.Frame)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // repeated .VSSRef.Robot robots = 2;
  {
    unsigned int count = static_cast<unsigned int>(this->robots_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->robots(static_cast<int>(i)));
    }
  }

  // .VSSRef.Color teamColor = 1;
  if (this->teamcolor() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::EnumSize(this->teamcolor());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Frame::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:VSSRef.Frame)
  GOOGLE_DCHECK_NE(&from, this);
  const Frame* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Frame>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:VSSRef.Frame)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:VSSRef.Frame)
    MergeFrom(*source);
  }
}

void Frame::MergeFrom(const Frame& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:VSSRef.Frame)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  robots_.MergeFrom(from.robots_);
  if (from.teamcolor() != 0) {
    set_teamcolor(from.teamcolor());
  }
}

void Frame::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:VSSRef.Frame)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Frame::CopyFrom(const Frame& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:VSSRef.Frame)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Frame::IsInitialized() const {
  return true;
}

void Frame::Swap(Frame* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Frame::InternalSwap(Frame* other) {
  using std::swap;
  CastToBase(&robots_)->InternalSwap(CastToBase(&other->robots_));
  swap(teamcolor_, other->teamcolor_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Frame::GetMetadata() const {
  protobuf_vssref_5fcommon_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_vssref_5fcommon_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace VSSRef
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::VSSRef::Robot* Arena::CreateMaybeMessage< ::VSSRef::Robot >(Arena* arena) {
  return Arena::CreateInternal< ::VSSRef::Robot >(arena);
}
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::VSSRef::Frame* Arena::CreateMaybeMessage< ::VSSRef::Frame >(Arena* arena) {
  return Arena::CreateInternal< ::VSSRef::Frame >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
