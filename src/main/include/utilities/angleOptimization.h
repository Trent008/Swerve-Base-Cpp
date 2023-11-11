#pragma once

// return the sum of angle1 and angle2 limited to -180 to 180 degrees
float angleSum(float const &angle1, float const &angle2) {
    float res = angle1 + angle2;
    res += (res<-180) ? 360 : (res>180) ? -360 : 0;
    return res;
}

// return the difference between angle1 and angle2 limited to -180 to 180 degrees
float angleDifference(float const &angle1, float const &angle2) {
    float res = angle1 - angle2;
    res += (res<-180) ? 360 : (res>180) ? -360 : 0;
    return res;
}