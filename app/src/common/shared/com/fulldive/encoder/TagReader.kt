package com.fulldive.encoder

@ByteKey(bytes = [0x57, 0x70, 0x41, 0x67, 0x52, 0x68, 0x78, 0x53, 0x73, 0x70, 0x6e, 0x32, 0x6d, 0x32, 0x42, 0x46])
class TagReader {
    private val map = mutableMapOf<String, String>()
    private val aesUtils by lazy {
        val bytes = this::class.java.getAnnotation(ByteKey::class.java)?.bytes
            ?: byteArrayOf(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        AESUtils(bytes)
    }

    fun encrypt(text: String): String {
        val encryptedText = aesUtils.encrypt(text)
        return encryptedText;
    }

    fun read(text: String): String {
        var result = text
        try {
            result = map[text]?:let {
                aesUtils.read(text).also { value ->
                    map[text] = value
                }
            }
        } catch (ex: Exception) {
            ex.printStackTrace()
        }
        return result
    }
    companion object{
        val instance by lazy { TagReader() }
    }
}
