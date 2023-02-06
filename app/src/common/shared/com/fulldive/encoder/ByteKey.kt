package com.fulldive.encoder

@Target(AnnotationTarget.CLASS)
@Retention(AnnotationRetention.RUNTIME)
annotation class ByteKey(val bytes: ByteArray)
