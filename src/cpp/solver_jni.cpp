#include <jni.h>
#include "solver.h"   // Your Solver class header
#include <memory>

extern "C" {

JNIEXPORT jlong JNICALL Java_sigmacorns_Solver_nativeCreate(JNIEnv* env, jobject thiz) {
    try {
        Solver* solver = new Solver();
        return reinterpret_cast<jlong>(solver);
    } catch (const std::exception& e) {
        // throw Java exception on failure
        jclass exClass = env->FindClass("java/lang/RuntimeException");
        env->ThrowNew(exClass, e.what());
        return 0;
    }
}

JNIEXPORT jint JNICALL Java_sigmacorns_Solver_nativeSolve(JNIEnv* env, jobject thiz, jlong nativeHandle,
                                                                     jdoubleArray jtarget,
                                                                     jdoubleArray jx0,
                                                                     jdoubleArray juLast,
                                                                     jdoubleArray jp,
                                                                     jdoubleArray jxGuess,
                                                                     jdoubleArray juGuess,
                                                                     jdoubleArray jxOut,
                                                                     jdoubleArray juOut) {
    if (nativeHandle == 0) {
        jclass exClass = env->FindClass("java/lang/IllegalStateException");
        env->ThrowNew(exClass, "Native Solver not initialized");
        return -1;
    }

    Solver* solver = reinterpret_cast<Solver*>(nativeHandle);

    // Get raw pointers from Java arrays
    jdouble* t = env->GetDoubleArrayElements(jtarget, nullptr);
    jdouble* x0 = env->GetDoubleArrayElements(jx0, nullptr);
    jdouble* uLast = env->GetDoubleArrayElements(juLast, nullptr);
    jdouble* p = env->GetDoubleArrayElements(jp, nullptr);
    jdouble* xGuess = env->GetDoubleArrayElements(jxGuess, nullptr);
    jdouble* uGuess = env->GetDoubleArrayElements(juGuess, nullptr);
    jdouble* xOut = env->GetDoubleArrayElements(jxOut, nullptr);
    jdouble* uOut = env->GetDoubleArrayElements(juOut, nullptr);

    int result = solver->solve(t, x0, uLast, p, xGuess, uGuess, xOut, uOut);

    // Release arrays back to JVM
    env->ReleaseDoubleArrayElements(jx0, x0, JNI_ABORT);    // input arrays, no changes
    env->ReleaseDoubleArrayElements(juLast, uLast, JNI_ABORT);
    env->ReleaseDoubleArrayElements(jxOut, xOut, 0);        // output arrays, copy back changes
    env->ReleaseDoubleArrayElements(juOut, uOut, 0);

    return result;
}

JNIEXPORT void JNICALL Java_sigmacorns_Solver_nativeDestroy(JNIEnv* env, jobject thiz, jlong nativeHandle) {
    if (nativeHandle != 0) {
        Solver* solver = reinterpret_cast<Solver*>(nativeHandle);
        delete solver;
    }
}

} // extern "C"
