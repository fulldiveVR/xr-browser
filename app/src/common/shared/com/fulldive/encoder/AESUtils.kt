package com.fulldive.encoder

import javax.crypto.Cipher
import javax.crypto.SecretKey
import javax.crypto.spec.SecretKeySpec

class AESUtils(private val keyValue: ByteArray) {

    fun encrypt(cleartext: String): String {
        return try {
            val rawKey: ByteArray = getRawKey()
            val result: ByteArray = encrypt(rawKey, cleartext.toByteArray())
            toHex(result)
        } catch (ex: Exception) {
            ex.printStackTrace()
            ""
        }
    }

    fun read(text: String): String {
        return try {
            val bytes = toBytes(text)
            val result = read(bytes)
            String(result)
        } catch (ex: Exception) {
            ex.printStackTrace()
            ""
        }
    }

    @Throws(java.lang.Exception::class)
    private fun read(encrypted: ByteArray): ByteArray {
        val skeySpec: SecretKey = SecretKeySpec(keyValue, "AES")
        val cipher = Cipher.getInstance("AES")
        cipher.init(Cipher.DECRYPT_MODE, skeySpec)
        return cipher.doFinal(encrypted)
    }

    private fun toBytes(hexString: String): ByteArray {
        val len = hexString.length / 2
        val result = ByteArray(len)
        for (i in 0 until len) result[i] =
            Integer.valueOf(hexString.substring(2 * i, 2 * i + 2), 16).toByte()
        return result
    }

    @Throws(java.lang.Exception::class)
    private fun getRawKey(): ByteArray {
        val key: SecretKey = SecretKeySpec(keyValue, "AES")
        return key.encoded
    }

    @Throws(java.lang.Exception::class)
    private fun encrypt(raw: ByteArray, clear: ByteArray): ByteArray {
        val skeySpec: SecretKey = SecretKeySpec(raw, "AES")
        val cipher = Cipher.getInstance("AES")
        cipher.init(Cipher.ENCRYPT_MODE, skeySpec)
        return cipher.doFinal(clear)
    }
    companion object {
        private val HEX = "0123456789ABCDEF"

        fun toHex(buf: ByteArray?): String {
            if (buf == null) return ""
            val result = StringBuffer(2 * buf.size)
            for (i in buf.indices) {
                appendHex(result, buf[i])
            }
            return result.toString()
        }

        private fun appendHex(sb: StringBuffer, b: Byte) {
            sb.append(HEX[b.toInt() shr 4 and 0x0f]).append(HEX[b.toInt() and 0x0f])
        }
    }
}

