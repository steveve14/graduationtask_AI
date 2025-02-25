plugins {
    alias(libs.plugins.android.application)
}

android {
    namespace = "com.example.movedistance"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.example.movedistance"
        minSdk = 33
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }

}

dependencies {
    implementation(libs.appcompat)
    implementation(libs.material)
    implementation(libs.activity)
    implementation(libs.constraintlayout)
    implementation(libs.play.services.location)
    testImplementation(libs.junit)
    androidTestImplementation(libs.ext.junit)
    androidTestImplementation(libs.espresso.core)

    //math
    implementation("org.apache.commons:commons-math3:3.6.1")
    //AI 작동
    implementation("org.pytorch:pytorch_android:1.10.0")
    implementation("org.pytorch:pytorch_android_torchvision:1.10.0")

    //지도
    implementation("org.osmdroid:osmdroid-android:6.1.20")
    implementation("com.github.MKergall:osmbonuspack:6.9.0")


}