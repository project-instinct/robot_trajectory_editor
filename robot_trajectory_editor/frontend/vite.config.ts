import { defineConfig } from 'vitest/config'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/api': 'http://127.0.0.1:5000',
    },
  },
  test: {
    environment: 'node',
    include: ['src/__tests__/**/*.test.ts'],
  },
})
