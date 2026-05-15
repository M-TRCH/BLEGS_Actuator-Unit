

============================================================
// โค้ด C/C++ สำหรับนำไปฝังในฟังก์ชัน Inverse Kinematics ของ STM32
// คัดลอกส่วนนี้ไปใช้งานได้เลยครับ
============================================================

// ฟังก์ชันชดเชยความคลาดเคลื่อน (Data-driven Kinematic Compensation)
void apply_kinematic_compensation(float thetaA_deg, float thetaB_deg, float* target_x_mm, float* target_y_mm) {
    
    // คำนวณตัวแปรพหุนาม
    float tA = thetaA_deg;
    float tB = thetaB_deg;
    float tA2 = tA * tA;
    float tB2 = tB * tB;
    float tAB = tA * tB;

    // คำนวณ Error แกน X (mm) จากสมการที่เทรนมาได้
    float err_x = -10.724399f 
                + (-0.053052f * tA) 
                + (-0.100359f * tB) 
                + ( 0.000185f * tA2) 
                + (-0.001052f * tAB) 
                + ( 0.000016f * tB2);

    // คำนวณ Error แกน Y (mm) จากสมการที่เทรนมาได้
    float err_y =  14.230538f 
                + ( 0.037770f * tA) 
                + ( 0.477321f * tB) 
                + (-0.000356f * tA2) 
                + ( 0.002805f * tAB) 
                + ( 0.001469f * tB2);

    // ปรับแก้พิกัดเป้าหมาย (นำเป้าหมายอุดมคติ มาลบด้วย Error ที่คาดการณ์)
    *target_x_mm = *target_x_mm - err_x;
    *target_y_mm = *target_y_mm - err_y;
}
