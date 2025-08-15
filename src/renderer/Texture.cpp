#include "Texture.hpp"

CTexture::CTexture() {
    ; // naffin'
}

CTexture::~CTexture() {
    destroyTexture();
}

void CTexture::destroyTexture() {
    if (m_bAllocated) {
        glDeleteTextures(1, &m_iTexID);
        m_iTexID = 0;
    }
    m_bAllocated = false;
}

void CTexture::allocate() {
    if (!m_bAllocated) {
        glGenTextures(1, &m_iTexID);
        // Verify texture was generated successfully
        if (m_iTexID == 0) {
            // Try again in case of transient failure
            glGenTextures(1, &m_iTexID);
        }
        m_bAllocated = (m_iTexID != 0);
    }
}
