/*
 * test_encoding.c
 * Round-trip encode/decode tests for all ODID message types.
 * Uses opendroneid-core-c directly, no MAVLink or hardware required.
 */

#include <opendroneid.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); \
        return 1; \
    } \
    printf("PASS: %s\n", msg); \
} while (0)

static int test_basic_id(void) {
    ODID_BasicID_data in = {0};
    in.IDType = ODID_IDTYPE_SERIAL_NUMBER;
    in.UAType = ODID_UATYPE_AEROPLANE;  /* Fixed wing = AEROPLANE in this library */
    strncpy(in.UASID, "SWE-SSRS-00000000001", sizeof(in.UASID) - 1);
    in.UASID[sizeof(in.UASID) - 1] = '\0';

    ODID_BasicID_encoded enc;
    CHECK(encodeBasicIDMessage(&enc, &in) == ODID_SUCCESS, "encodeBasicIDMessage");

    ODID_BasicID_data out = {0};
    CHECK(decodeBasicIDMessage(&out, &enc) == ODID_SUCCESS, "decodeBasicIDMessage");

    CHECK(out.IDType == in.IDType, "BasicID IDType round-trip");
    CHECK(out.UAType == in.UAType, "BasicID UAType round-trip");
    CHECK(strncmp(out.UASID, in.UASID, sizeof(in.UASID)) == 0, "BasicID UASID round-trip");
    return 0;
}

static int test_location(void) {
    ODID_Location_data in = {0};
    in.Status           = ODID_STATUS_AIRBORNE;
    in.Latitude         = 57.7089;   /* Gothenburg */
    in.Longitude        = 11.9746;
    in.AltitudeGeo      = 150.0f;
    in.AltitudeBaro     = 148.0f;
    in.Height           = 100.0f;
    in.HeightType       = ODID_HEIGHT_REF_OVER_TAKEOFF;
    in.SpeedHorizontal  = 25.0f;    /* m/s */
    in.SpeedVertical    = 0.5f;
    in.Direction        = 270.0f;   /* west */
    in.HorizAccuracy    = ODID_HOR_ACC_10_METER;
    in.VertAccuracy     = ODID_VER_ACC_10_METER;
    in.BaroAccuracy     = ODID_VER_ACC_10_METER;
    in.SpeedAccuracy    = ODID_SPEED_ACC_1_METERS_PER_SECOND;
    in.TSAccuracy       = ODID_TIME_ACC_0_1_SECOND;
    in.TimeStamp        = 1234.5f;

    ODID_Location_encoded enc;
    CHECK(encodeLocationMessage(&enc, &in) == ODID_SUCCESS, "encodeLocationMessage");

    ODID_Location_data out = {0};
    CHECK(decodeLocationMessage(&out, &enc) == ODID_SUCCESS, "decodeLocationMessage");

    CHECK(out.Status == in.Status, "Location Status round-trip");
    CHECK(fabs(out.Latitude  - in.Latitude)  < 1e-4, "Location Latitude round-trip");
    CHECK(fabs(out.Longitude - in.Longitude) < 1e-4, "Location Longitude round-trip");
    CHECK(fabsf(out.SpeedHorizontal - in.SpeedHorizontal) < 0.5f, "Location SpeedH round-trip");
    return 0;
}

static int test_system(void) {
    ODID_System_data in = {0};
    in.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
    in.ClassificationType   = ODID_CLASSIFICATION_TYPE_EU;
    in.OperatorLatitude     = 57.7089;
    in.OperatorLongitude    = 11.9746;
    in.OperatorAltitudeGeo  = 10.0f;
    in.CategoryEU           = ODID_CATEGORY_EU_CERTIFIED;
    in.ClassEU              = ODID_CLASS_EU_CLASS_4;
    in.Timestamp            = 200000000.0f;

    ODID_System_encoded enc;
    CHECK(encodeSystemMessage(&enc, &in) == ODID_SUCCESS, "encodeSystemMessage");

    ODID_System_data out = {0};
    CHECK(decodeSystemMessage(&out, &enc) == ODID_SUCCESS, "decodeSystemMessage");

    CHECK(out.ClassificationType == in.ClassificationType, "System ClassificationType round-trip");
    CHECK(out.CategoryEU         == in.CategoryEU,         "System CategoryEU round-trip");
    CHECK(out.ClassEU            == in.ClassEU,            "System ClassEU round-trip");
    return 0;
}

static int test_operator_id(void) {
    ODID_OperatorID_data in = {0};
    in.OperatorIdType = ODID_OPERATOR_ID;   /* only value defined in ASTM F3411 */
    strncpy(in.OperatorId, "SWE-OP-12345", sizeof(in.OperatorId) - 1);

    ODID_OperatorID_encoded enc;
    CHECK(encodeOperatorIDMessage(&enc, &in) == ODID_SUCCESS, "encodeOperatorIDMessage");

    ODID_OperatorID_data out = {0};
    CHECK(decodeOperatorIDMessage(&out, &enc) == ODID_SUCCESS, "decodeOperatorIDMessage");

    CHECK(out.OperatorIdType == in.OperatorIdType, "OperatorID type round-trip");
    CHECK(strncmp(out.OperatorId, in.OperatorId, sizeof(in.OperatorId)) == 0,
          "OperatorID id round-trip");
    return 0;
}

static int test_self_id(void) {
    ODID_SelfID_data in = {0};
    in.DescType = ODID_DESC_TYPE_TEXT;
    strncpy(in.Desc, "Sea rescue surveillance", sizeof(in.Desc) - 1);
    in.Desc[sizeof(in.Desc) - 1] = '\0';

    ODID_SelfID_encoded enc;
    CHECK(encodeSelfIDMessage(&enc, &in) == ODID_SUCCESS, "encodeSelfIDMessage");

    ODID_SelfID_data out = {0};
    CHECK(decodeSelfIDMessage(&out, &enc) == ODID_SUCCESS, "decodeSelfIDMessage");

    CHECK(out.DescType == in.DescType, "SelfID DescType round-trip");
    CHECK(strncmp(out.Desc, in.Desc, sizeof(in.Desc)) == 0, "SelfID Desc round-trip");
    return 0;
}

static int test_message_pack(void) {
    ODID_MessagePack_data pack = {0};
    pack.SingleMessageSize = ODID_MESSAGE_SIZE;
    pack.MsgPackSize = 3;

    ODID_BasicID_data basic = {0};
    basic.IDType = ODID_IDTYPE_SERIAL_NUMBER;
    basic.UAType = ODID_UATYPE_AEROPLANE;
    strncpy(basic.UASID, "TEST01", sizeof(basic.UASID) - 1);
    encodeBasicIDMessage((ODID_BasicID_encoded *)pack.Messages[0].rawData, &basic);

    ODID_Location_data loc = {0};
    loc.Status    = ODID_STATUS_AIRBORNE;
    loc.Latitude  = 57.7;
    loc.Longitude = 12.0;
    loc.AltitudeGeo = 100.0f;
    encodeLocationMessage((ODID_Location_encoded *)pack.Messages[1].rawData, &loc);

    ODID_OperatorID_data op = {0};
    op.OperatorIdType = ODID_OPERATOR_ID;
    strncpy(op.OperatorId, "SWE-OP-1", sizeof(op.OperatorId) - 1);
    encodeOperatorIDMessage((ODID_OperatorID_encoded *)pack.Messages[2].rawData, &op);

    ODID_MessagePack_encoded enc;
    CHECK(encodeMessagePack(&enc, &pack) == ODID_SUCCESS, "encodeMessagePack");

    CHECK(enc.MsgPackSize == 3, "MessagePack count preserved");
    printf("MessagePack encoded: %d messages × %d bytes\n",
           enc.MsgPackSize, ODID_MESSAGE_SIZE);
    return 0;
}

int main(void) {
    int failures = 0;
    failures += test_basic_id();
    failures += test_location();
    failures += test_system();
    failures += test_operator_id();
    failures += test_self_id();
    failures += test_message_pack();

    if (failures == 0)
        printf("\nAll encoding tests passed.\n");
    else
        printf("\n%d test(s) FAILED.\n", failures);

    return failures ? 1 : 0;
}
