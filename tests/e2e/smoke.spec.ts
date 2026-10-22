import { test, expect } from '@playwright/test';

test('loads arm UI and responds to keyboard nudge', async ({ page }) => {
  const errors: string[] = [];
  page.on('console', (msg) => {
    if (msg.type() === 'error') {
      errors.push(msg.text());
    }
  });

  const response = await page.goto('http://localhost:8000/?test=1');
  expect(response?.status()).toBe(200);

  await page.waitForFunction(() => (window as any).__TEST_HOOKS__?.getLinkCount() >= 6, null, {
    timeout: 15000,
  });

  expect(errors).toEqual([]);

  await page.evaluate(() => (window as any).__TEST_HOOKS__.requestControl());
  await page.waitForTimeout(300);
  const before = await page.evaluate(() => (window as any).__TEST_HOOKS__.getJointPositions());
  await page.evaluate(() => {
    (window as any).__TEST_HOOKS__.pressKey('Shift');
    (window as any).__TEST_HOOKS__.nudgeTargetZ(0.05);
  });
  await page.waitForTimeout(1500);
  const after = await page.evaluate(() => (window as any).__TEST_HOOKS__.getJointPositions());
  expect(after).not.toEqual(before);
});
