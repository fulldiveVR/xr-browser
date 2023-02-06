package mozilla.components.support.base.log.logger

class Logger(
    private val tag: String? = null
) {
    fun debug(message: String? = null, throwable: Throwable? = null) = Unit

    fun info(message: String? = null, throwable: Throwable? = null) = Unit

    fun warn(message: String? = null, throwable: Throwable? = null) = Unit

    fun error(message: String? = null, throwable: Throwable? = null) = Unit

    fun measure(message: String, block: () -> Unit) = Unit

    fun hooked() = Unit

    companion object {
        fun debug(message: String? = null, throwable: Throwable? = null) = Unit

        fun info(message: String? = null, throwable: Throwable? = null) = Unit

        fun warn(message: String? = null, throwable: Throwable? = null) = Unit

        fun error(message: String? = null, throwable: Throwable? = null) = Unit

        fun measure(message: String, block: () -> Unit) = Unit
    }
}